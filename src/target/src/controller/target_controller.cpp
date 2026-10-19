#include <target/controller/target_controller.hpp>

#include "calculation/association/detection_matcher.hpp"
#include "calculation/tracking/constant_acceleration_algorithm.hpp"
#include "domain/target.hpp"
#include "repository/target_repository.hpp"
#include "selector/tentative_selector.hpp"
#include "selector/track_selector.hpp"
#include "math/trigonometry.hpp"

#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>

namespace target
{
namespace
{

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
}

std::vector<Target> toTargets(
    const DetectionEvent& detection,
    const std::shared_ptr<const ITrackingAlgorithm>& trackingAlgorithm)
{
    std::vector<Target> targets;
    targets.reserve(detection.plots.size());
    for (const auto& plot : detection.plots)
    {
        targets.emplace_back(math::SphericalPosition(
                plot.slantRange, math::SphericalDirection(plot.azimuth, plot.elevation)),
            plot.dopplerVelocity, plot.power_db, detection.transmitTime, trackingAlgorithm);
    }
    return targets;
}

std::optional<beam::BeamRequest> trackingRequestOrRemove(
    TargetRepository& repository,
    const Target& target,
    std::chrono::milliseconds transmitTime)
{
    const auto prediction = target.predict(transmitTime);
    if (!prediction)
    {
        repository.remove(target.id());
        return std::nullopt;
    }

    return beam::BeamRequest(
        beam::BeamRequest::BeamType::TRACKING,
        prediction->timestamp,
        target.id(),
        prediction->sphericalPosition.direction.azimuth,
        prediction->sphericalPosition.direction.elevation);
}

TrackEvent trackEvent(const Target& target)
{
    if (target.id() > std::numeric_limits<std::uint16_t>::max())
    {
        throw std::overflow_error("Target ID cannot be represented by TrackEvent");
    }

    const auto& state = *target.trackingState();
    using namespace math::literals;

    const auto& position = state.position;
    const auto& velocity = state.velocity;
    const auto horizontalRange = position.horizontalDistance();
    const auto horizontalSpeed = velocity.horizontalSpeed();
    const auto speed = velocity.speed();
    const math::Angle heading = horizontalSpeed.greaterThan(0.0_mps, math::LinearVelocity::NONE_TOLERANCE)
        ? math::atan2(velocity.x, velocity.y)
        : 0_deg;
    const math::Angle flightPathAngle = speed.greaterThan(0.0_mps, math::LinearVelocity::NONE_TOLERANCE)
        ? math::atan2(velocity.z, horizontalSpeed)
        : 0_deg;
    return TrackEvent(
        static_cast<std::uint16_t>(target.id()),
        TrackEvent::TrackState::TRACKING,
        target.slantRange(),
        horizontalRange,
        target.azimuth(),
        target.elevation(),
        target.dopplerVelocity(),
        0_deg,
        0_deg,
        position.z,
        speed,
        heading,
        flightPathAngle,
        target.lastObservationTime());
}

} // namespace

TargetController::TargetController(const TargetConfig& config)
    : _config(std::make_shared<const TargetConfig>(config))
    , _trackingAlgorithm(std::make_shared<ConstantAccelerationAlgorithm>(*_config))
    , _targetRepository(std::make_shared<TargetRepository>())
    , _matcher(std::make_shared<DetectionMatcher>(*_config))
    , _tentativeSelector(std::make_shared<TentativeSelector>(
          *_targetRepository, *_matcher))
    , _trackSelector(std::make_shared<TrackSelector>(
          *_targetRepository, *_matcher))
{
}

std::optional<beam::BeamRequest> TargetController::handleDetection(DetectionEvent detection)
{
    const auto& config = *_config;
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

        const auto transmitTime = addTime(resultTime, config.confirmationDelay);
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

        const auto transmitTime = addTime(resultTime, config.trackingInterval);
        if (!target->confirm(receivedTargets[*selected]))
        {
            _targetRepository->remove(requestId);
            return std::nullopt;
        }
        return trackingRequestOrRemove(*_targetRepository, *target, transmitTime);
    }

    if (!selected
        && target->consecutiveMissCount() >= config.maxConsecutiveTrackingMisses - 1)
    {
        target->recordTrackingMiss(resultTime);
        _targetRepository->remove(requestId);
        return std::nullopt;
    }

    const auto transmitTime = addTime(resultTime, config.trackingInterval);
    const bool updated = selected && target->update(receivedTargets[*selected]);
    if (!updated)
    {
        target->recordTrackingMiss(resultTime);
        if (target->consecutiveMissCount() >= config.maxConsecutiveTrackingMisses)
        {
            _targetRepository->remove(requestId);
            return std::nullopt;
        }
    }
    // Predict again at the next transmit time, even when no measurement matched.
    return trackingRequestOrRemove(*_targetRepository, *target, transmitTime);
}

std::vector<TrackEvent> TargetController::nextTrackEvents() const
{
    std::vector<TrackEvent> events;
    for (const auto& target : _targetRepository->targets())
    {
        if (target.state() == TargetState::TRACKING)
        {
            events.push_back(trackEvent(target));
            if (events.size() == _config->maxReportedTrackCount)
            {
                break;
            }
        }
    }
    return events;
}

} // namespace target
