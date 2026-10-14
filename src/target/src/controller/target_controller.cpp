#include <target/controller/target_controller.hpp>

#include "calculation/association/detection_matcher.hpp"
#include "calculation/tracking/cartesian_math.hpp"
#include "calculation/tracking/constant_velocity_algorithm.hpp"
#include "domain/target.hpp"
#include "repository/target_repository.hpp"
#include "selector/tentative_selector.hpp"
#include "selector/track_selector.hpp"
#include "validation/detection_validation.hpp"

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

    // Validate the entire external message before converting or storing any target.
    for (const auto& plot : detection.plots)
    {
        if (!validation::hasValidMeasurement(
                plot.slantRange, plot.azimuth, plot.elevation, plot.dopplerVelocity))
        {
            throw std::invalid_argument("Invalid beam detection plot");
        }
    }
}

std::vector<Target> toTargets(
    const DetectionEvent& detection,
    const std::shared_ptr<const ITrackingAlgorithm>& trackingAlgorithm)
{
    std::vector<Target> targets;
    targets.reserve(detection.plots.size());
    for (const auto& plot : detection.plots)
    {
        targets.emplace_back(plot.slantRange, plot.azimuth, plot.elevation,
            plot.dopplerVelocity, plot.power_db, detection.transmitTime, trackingAlgorithm);
    }
    return targets;
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
    using namespace math::literals;

    const auto& position = state.position;
    const auto& velocity = state.velocity;
    const auto horizontalRange = math::Distance::fromMeters(
        math::Vector{position.x.m(), position.y.m()}.norm());
    const auto horizontalSpeed = math::LinearVelocity::fromMetersPerSecond(
        math::Vector{velocity.x.mps(), velocity.y.mps()}.norm());
    const auto speed = math::LinearVelocity::fromMetersPerSecond(cartesian::toVector(velocity).norm());
    // Report position and motion in the same fixed radar frame as the detections.
    // Latitude/longitude are unavailable; a stationary target has no motion angle.
    const std::optional<math::Angle> heading = horizontalSpeed > 0.0_mps
        ? std::optional{math::Angle::fromRadians(std::atan2(velocity.y.mps(), velocity.x.mps()))}
        : std::nullopt;
    const std::optional<math::Angle> flightPathAngle = speed > 0.0_mps
        ? std::optional{math::Angle::fromRadians(std::atan2(velocity.z.mps(), horizontalSpeed.mps()))}
        : std::nullopt;
    return TrackEvent(
        static_cast<std::uint16_t>(target.id()),
        TrackEvent::TrackState::TRACKING,
        target.slantRange(),
        horizontalRange,
        target.azimuth(),
        target.elevation(),
        target.dopplerVelocity(),
        std::nullopt,
        std::nullopt,
        position.z,
        speed,
        heading,
        flightPathAngle,
        target.lastObservationTime());
}

} // namespace

TargetController::TargetController()
    : _trackingAlgorithm(std::make_shared<ConstantVelocityAlgorithm>())
    , _targetRepository(std::make_shared<TargetRepository>())
    , _matcher(std::make_shared<DetectionMatcher>())
    , _tentativeSelector(std::make_shared<TentativeSelector>(
          *_targetRepository, *_matcher))
    , _trackSelector(std::make_shared<TrackSelector>(
          *_targetRepository, *_matcher))
{
}

std::optional<beam::BeamRequest> TargetController::handleDetection(DetectionEvent detection)
{
    validateDetection(detection);
    const auto receivedTargets = toTargets(detection, _trackingAlgorithm);
    // Beam metadata belongs to the result, not to an individual physical target.
    const auto beamType = detection.beamType;
    const auto requestId = detection.requestId;
    const auto resultTime = detection.transmitTime;

    using BeamType = beam::BeamCommand::BeamType;
    if (beamType == BeamType::SEARCH)
    {
        const auto previous = _targetRepository->lastSearchResultTime(requestId);
        if (previous && resultTime <= *previous)
        {
            return std::nullopt;
        }

        const auto selected = _tentativeSelector->selectNewCandidate(receivedTargets);
        if (!selected)
        {
            _targetRepository->markSearchResultProcessed(requestId, resultTime);
            return std::nullopt;
        }

        const auto transmitTime = addTime(resultTime, CONFIRMATION_DELAY);
        const Target& candidate = receivedTargets[*selected];
        const auto id = _targetRepository->add(candidate);
        _targetRepository->markSearchResultProcessed(requestId, resultTime);
        return beam::BeamRequest(
            beam::BeamRequest::BeamType::CONFIRMATION,
            transmitTime,
            id,
            candidate.azimuth(),
            candidate.elevation());
    }

    Target* target = _targetRepository->find(requestId);
    const auto expectedState = beamType == BeamType::CONFIRMATION
        ? TargetState::TENTATIVE : TargetState::TRACKING;
    if (!target || target->state() != expectedState || resultTime <= target->lastResultTime())
    {
        return std::nullopt;
    }

    const auto selected = _trackSelector->selectObservation(requestId, resultTime, receivedTargets);
    if (beamType == BeamType::CONFIRMATION)
    {
        if (!selected)
        {
            _targetRepository->remove(requestId);
            return std::nullopt;
        }

        const auto transmitTime = addTime(resultTime, TRACKING_INTERVAL);
        if (!target->confirm(receivedTargets[*selected]))
        {
            _targetRepository->remove(requestId);
            return std::nullopt;
        }
        const auto request = trackingRequest(*target, transmitTime);
        if (!request)
        {
            _targetRepository->remove(requestId);
        }
        return request;
    }

    if (!selected
        && target->consecutiveMissCount() >= MAX_CONSECUTIVE_TRACKING_MISSES - 1)
    {
        target->recordTrackingMiss(resultTime);
        _targetRepository->remove(requestId);
        return std::nullopt;
    }

    const auto transmitTime = addTime(resultTime, TRACKING_INTERVAL);
    const bool updated = selected && target->update(receivedTargets[*selected]);
    if (!updated)
    {
        target->recordTrackingMiss(resultTime);
        if (target->consecutiveMissCount() >= MAX_CONSECUTIVE_TRACKING_MISSES)
        {
            _targetRepository->remove(requestId);
            return std::nullopt;
        }
    }
    // Predict again at the next transmit time, even when no measurement matched.
    const auto request = trackingRequest(*target, transmitTime);
    if (!request)
    {
        _targetRepository->remove(requestId);
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
