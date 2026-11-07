#include "tentative_track.hpp"

#include "domain/tracking_algorithm.hpp"

#include <stdexcept>
#include <utility>

namespace target
{

TentativeTrack::TentativeTrack(
    std::uint32_t id,
    Observation observation,
    std::shared_ptr<const ITrackingAlgorithm> trackingAlgorithm
) : _id(id),
    _observation(std::move(observation)),
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

std::chrono::milliseconds TentativeTrack::lastResultTime() const
{
    return _observation.lastObservationTime();
}

std::optional<TrackState> TentativeTrack::initialize(const Observation& observation) const
{
    const auto time = observation.lastObservationTime();
    if (time <= lastResultTime())
    {
        return std::nullopt;
    }

    auto state = _trackingAlgorithm->initialize(_observation, observation);
    if (state && state->timestamp() != time)
    {
        throw std::invalid_argument("Tracking state must use the observation time");
    }
    return state;
}

std::optional<ConfirmedTrack> TentativeTrack::confirm(const Observation& observation) const
{
    auto state = initialize(observation);
    if (!state)
    {
        return std::nullopt;
    }
    return ConfirmedTrack(_id, lastResultTime(), std::move(*state), _trackingAlgorithm);
}

} // namespace target
