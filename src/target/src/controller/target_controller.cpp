#include <target/controller/target_controller.hpp>

#include "calculation/association/detection_matcher.hpp"
#include "calculation/tracking/constant_velocity_algorithm.hpp"
#include "domain/target.hpp"
#include "repository/target_repository.hpp"
#include "selector/tentative_selector.hpp"
#include "selector/track_selector.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>

namespace target
{
namespace
{

constexpr std::chrono::milliseconds CONFIRMATION_DELAY{100};
constexpr std::chrono::milliseconds TRACKING_INTERVAL{1000};
constexpr std::uint32_t MAX_CONSECUTIVE_TRACKING_MISSES = 3;

std::chrono::milliseconds addTime(
    std::chrono::milliseconds timestamp,
    std::chrono::milliseconds interval)
{
    const auto maximum = std::numeric_limits<std::chrono::milliseconds::rep>::max();
    if (timestamp.count() > maximum - interval.count())
    {
        throw std::overflow_error("Requested beam transmit time overflow");
    }
    return timestamp + interval;
}

void validateDetection(const DetectionEvent& detection)
{
    using BeamType = beam::BeamCommand::BeamType;
    if (detection.transmitTime.count() < 0
        || (detection.beamType != BeamType::SEARCH
            && detection.beamType != BeamType::CONFIRMATION
            && detection.beamType != BeamType::TRACKING)
        || (detection.beamType != BeamType::SEARCH && detection.requestId == 0))
    {
        throw std::invalid_argument("Invalid beam detection metadata");
    }

    const DetectionMatcher matcher;
    for (const auto& plot : detection.plots)
    {
        if (!matcher.isValidPosition(plot))
        {
            throw std::invalid_argument("Invalid beam detection plot");
        }
    }
}

std::optional<beam::BeamRequest> trackingRequest(
    const Target& target,
    std::chrono::milliseconds transmitTime)
{
    const auto prediction = target.predict(transmitTime);
    if (!prediction)
    {
        return std::nullopt;
    }

    return beam::BeamRequest(
        beam::BeamRequest::BeamType::TRACKING,
        prediction->timestamp,
        target.id(),
        prediction->azimuth,
        prediction->elevation);
}

TrackEvent trackEvent(const Target& target)
{
    if (target.id() > std::numeric_limits<std::uint16_t>::max())
    {
        throw std::overflow_error("Target ID cannot be represented by TrackEvent");
    }

    const auto& state = target.trackingState().value();
    const auto plot = target.lastPlot();
    const auto& position = state.positionMeters;
    const auto& velocity = state.velocityMetersPerSecond;
    const double horizontalRange = std::hypot(position[0], position[1]);
    const double horizontalSpeed = std::hypot(velocity[0], velocity[1]);
    const double speed = std::hypot(velocity[0], velocity[1], velocity[2]);
    // Report position and motion in the same fixed radar frame as the detections.
    // Latitude/longitude are unavailable; a stationary target has no motion angle.
    const double unavailable = std::numeric_limits<double>::quiet_NaN();
    const auto heading = horizontalSpeed > 0.0
        ? math::Angle::fromRadians(std::atan2(velocity[1], velocity[0]))
        : math::Angle::fromDegrees(unavailable);
    const auto flightPathAngle = speed > 0.0
        ? math::Angle::fromRadians(std::atan2(velocity[2], horizontalSpeed))
        : math::Angle::fromDegrees(unavailable);
    return TrackEvent(
        static_cast<std::uint16_t>(target.id()),
        TrackEvent::TrackState::TRACKING,
        plot.slantRange,
        math::Distance::fromMeters(horizontalRange),
        plot.azimuth,
        plot.elevation,
        plot.dopplerVelocity,
        math::Angle::fromDegrees(unavailable),
        math::Angle::fromDegrees(unavailable),
        math::Distance::fromMeters(position[2]),
        math::Velocity::fromMetersPerSecond(speed),
        heading,
        flightPathAngle,
        target.lastObservationTime());
}

} // namespace

TargetController::TargetController()
    : _targetRepository(std::make_shared<TargetRepository>(
        std::make_shared<ConstantVelocityAlgorithm>()))
{
}

std::optional<beam::BeamRequest> TargetController::handleDetection(DetectionEvent detection)
{
    validateDetection(detection);

    using BeamType = beam::BeamCommand::BeamType;
    if (detection.beamType == BeamType::SEARCH)
    {
        const auto previous = _targetRepository->lastSearchResultTime(detection.requestId);
        if (previous && detection.transmitTime <= *previous)
        {
            return std::nullopt;
        }

        const TentativeSelector selector(*_targetRepository);
        const auto selected = selector.selectNewCandidate(detection.plots);
        if (!selected)
        {
            _targetRepository->markSearchResultProcessed(
                detection.requestId, detection.transmitTime);
            return std::nullopt;
        }

        const auto transmitTime = addTime(detection.transmitTime, CONFIRMATION_DELAY);
        const auto& plot = detection.plots[*selected];
        const auto id = _targetRepository->add(plot, detection.transmitTime);
        _targetRepository->markSearchResultProcessed(detection.requestId, detection.transmitTime);
        return beam::BeamRequest(
            beam::BeamRequest::BeamType::CONFIRMATION,
            transmitTime,
            id,
            plot.azimuth,
            plot.elevation);
    }

    Target* target = _targetRepository->find(detection.requestId);
    const auto expectedState = detection.beamType == BeamType::CONFIRMATION
        ? TargetState::TENTATIVE : TargetState::TRACKING;
    if (!target || target->state() != expectedState
        || detection.transmitTime <= target->lastResultTime())
    {
        return std::nullopt;
    }

    const TrackSelector selector(*_targetRepository);
    const auto selected = selector.selectObservation(detection);
    if (detection.beamType == BeamType::CONFIRMATION)
    {
        if (!selected)
        {
            _targetRepository->remove(detection.requestId);
            return std::nullopt;
        }

        const auto transmitTime = addTime(detection.transmitTime, TRACKING_INTERVAL);
        if (!target->confirm(detection.plots[*selected], detection.transmitTime))
        {
            _targetRepository->remove(detection.requestId);
            return std::nullopt;
        }
        const auto request = trackingRequest(*target, transmitTime);
        if (!request)
        {
            _targetRepository->remove(detection.requestId);
        }
        return request;
    }

    if (!selected
        && target->consecutiveMissCount() >= MAX_CONSECUTIVE_TRACKING_MISSES - 1)
    {
        target->recordTrackingMiss(detection.transmitTime);
        _targetRepository->remove(detection.requestId);
        return std::nullopt;
    }

    const auto transmitTime = addTime(detection.transmitTime, TRACKING_INTERVAL);
    const bool updated = selected
        && target->update(detection.plots[*selected], detection.transmitTime);
    if (!updated)
    {
        target->recordTrackingMiss(detection.transmitTime);
        if (target->consecutiveMissCount() >= MAX_CONSECUTIVE_TRACKING_MISSES)
        {
            _targetRepository->remove(detection.requestId);
            return std::nullopt;
        }
    }
    // Predict again at the next transmit time, even when no measurement matched.
    const auto request = trackingRequest(*target, transmitTime);
    if (!request)
    {
        _targetRepository->remove(detection.requestId);
    }
    return request;
}

std::vector<TrackEvent> TargetController::nextTrackEvents() const
{
    std::vector<TrackEvent> events;
    for (const auto& target : _targetRepository->targets())
    {
        if (target.state() == TargetState::TRACKING)
        {
            events.push_back(trackEvent(target));
        }
    }
    return events;
}

} // namespace target
