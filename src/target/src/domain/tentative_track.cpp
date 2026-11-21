#include "tentative_track.hpp"

#include "domain/tracking/tracking_algorithm.hpp"

#include <stdexcept>
#include <utility>

namespace target
{

TentativeTrack::TentativeTrack(
    std::uint32_t id,
    Observation observation,
    TrackLifecycleConfig lifecycleConfig,
    std::shared_ptr<const ITrackingAlgorithm> trackingAlgorithm
) : _id(id),
    _observation(std::move(observation)),
    _lifecycleConfig(std::move(lifecycleConfig)),
    _trackingAlgorithm(std::move(trackingAlgorithm))
{
    if (_id == 0 || !_trackingAlgorithm)
    {
        throw std::invalid_argument("Tentative target requires a nonzero ID and tracking algorithm");
    }
}

std::uint32_t TentativeTrack::id() const
{
    return _id;
}

const Observation& TentativeTrack::observation() const
{
    return _observation;
}

bool TentativeTrack::canProcessResult(std::chrono::milliseconds resultTime) const
{
    return resultTime > _observation.observationTime();
}

bool TentativeTrack::isExpired(std::chrono::milliseconds currentTime) const
{
    return currentTime > _observation.observationTime() + _lifecycleConfig.confirmationDelay() + _lifecycleConfig.responseTimeout();
}

std::optional<TrackingResult> TentativeTrack::initialize(const Observation& observation) const
{
    const auto time = observation.observationTime();
    if (!canProcessResult(time))
    {
        return std::nullopt;
    }

    auto result = _trackingAlgorithm->initialize(_observation, observation);
    if (result && result->state.timestamp() != time)
    {
        throw std::invalid_argument("Tracking state must use the observation time");
    }
    return result;
}

std::optional<ConfirmedTrack> TentativeTrack::confirm(const Observation& observation) const
{
    auto result = initialize(observation);
    if (!result)
    {
        return std::nullopt;
    }
    auto& [state, algorithmState] = *result;
    return ConfirmedTrack(_id, _observation.observationTime(), std::move(state), _lifecycleConfig, _trackingAlgorithm, std::move(algorithmState));
}

} // namespace target
