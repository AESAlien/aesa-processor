#include "detection_handler.hpp"

#include "domain/factory/track_factory.hpp"
#include "repository/track_repository.hpp"
#include "domain/selector/tentative_selector.hpp"
#include "domain/selector/track_selector.hpp"

#include <limits>
#include <stdexcept>
#include <utility>

namespace target
{
namespace
{

std::chrono::milliseconds addTime(
    std::chrono::milliseconds timestamp,
    std::chrono::milliseconds interval
) {
    const auto maximum = std::numeric_limits<std::chrono::milliseconds::rep>::max();
    if (timestamp.count() > maximum - interval.count())
    {
        throw std::overflow_error("Requested beam transmit time overflow");
    }
    return timestamp + interval;
}

std::optional<beam::RequestBeamCommand> trackingRequestOrRemove(
    TrackRepository& repository,
    const ConfirmedTrack& target,
    std::chrono::milliseconds transmitTime
) {
    const auto prediction = target.predict(transmitTime);
    if (!prediction)
    {
        repository.remove(target.id());
        return std::nullopt;
    }

    return beam::RequestBeamCommand(
        beam::RequestBeamType::TRACKING,
        prediction->timestamp(),
        target.id(),
        prediction->sphericalPosition().direction
    );
}

} // namespace

DetectionHandler::DetectionHandler(
    TrackManagementConfig config,
    TrackRepository& repository,
    const TrackFactory& trackFactory,
    const TentativeSelector& tentativeSelector,
    const TrackSelector& trackSelector
) : _config(std::move(config)),
    _trackRepository(repository),
    _trackFactory(trackFactory),
    _tentativeSelector(tentativeSelector),
    _trackSelector(trackSelector)
{
}

std::optional<beam::RequestBeamCommand> DetectionHandler::handle(const DetectionResult& result)
{
    return std::visit(
        [this](const auto& value)
        {
            return handle(value);
        },
        result
    );
}

std::optional<beam::RequestBeamCommand> DetectionHandler::handle(
    const SearchDetectionResult& result
) {
    const auto previous = _lastSearchResultTimes.find(result.requestId);
    if (previous != _lastSearchResultTimes.end() && result.transmitTime <= previous->second)
    {
        return std::nullopt;
    }

    const auto selected = _tentativeSelector.selectNewCandidate(result.observations, _trackRepository.targets());
    if (!selected)
    {
        _lastSearchResultTimes.insert_or_assign(result.requestId, result.transmitTime);
        return std::nullopt;
    }

    const auto transmitTime = addTime(result.transmitTime, _config.confirmationDelay());
    const auto& candidate = result.observations[*selected];
    const auto id = _trackRepository.add(_trackFactory.createTentative(_trackRepository.nextId(), candidate));
    _lastSearchResultTimes.insert_or_assign(result.requestId, result.transmitTime);
    return beam::RequestBeamCommand(
        beam::RequestBeamType::CONFIRMATION,
        transmitTime,
        id,
        candidate.sphericalPosition().direction
    );
}

std::optional<beam::RequestBeamCommand> DetectionHandler::handle(
    const ConfirmationDetectionResult& result
) {
    const auto* tentative = _trackRepository.findTentative(result.requestId);
    if (!tentative || result.transmitTime <= tentative->lastResultTime())
    {
        return std::nullopt;
    }

    const auto selected = _trackSelector.selectConfirmation(
        *tentative,
        result.transmitTime,
        result.observations
    );
    if (!selected)
    {
        _trackRepository.remove(result.requestId);
        return std::nullopt;
    }

    const auto transmitTime = addTime(result.transmitTime, _config.trackingInterval());
    auto track = tentative->confirm(result.observations[*selected]);
    if (!track)
    {
        _trackRepository.remove(result.requestId);
        return std::nullopt;
    }
    if (!_trackRepository.replace(std::move(*track)))
    {
        return std::nullopt;
    }
    return trackingRequestOrRemove(
        _trackRepository,
        *_trackRepository.findTrack(result.requestId),
        transmitTime
    );
}

std::optional<beam::RequestBeamCommand> DetectionHandler::handle(
    const TrackingDetectionResult& result
) {
    ConfirmedTrack* track = _trackRepository.findTrack(result.requestId);
    if (!track || result.transmitTime <= track->lastResultTime())
    {
        return std::nullopt;
    }

    const auto selected = _trackSelector.selectObservation(
        *track,
        result.transmitTime,
        result.observations
    );
    if (!selected &&
        track->consecutiveMissCount() >= _config.maxConsecutiveTrackingMisses() - 1)
    {
        _trackRepository.remove(result.requestId);
        return std::nullopt;
    }

    const auto transmitTime = addTime(result.transmitTime, _config.trackingInterval());
    const bool updated = selected && track->update(result.observations[*selected]);
    if (!updated)
    {
        track->recordTrackingMiss(result.transmitTime);
        if (track->consecutiveMissCount() >= _config.maxConsecutiveTrackingMisses())
        {
            _trackRepository.remove(result.requestId);
            return std::nullopt;
        }
    }
    return trackingRequestOrRemove(_trackRepository, *track, transmitTime);
}

} // namespace target
