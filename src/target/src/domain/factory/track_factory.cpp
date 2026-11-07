#include "track_factory.hpp"

#include <stdexcept>
#include <utility>

namespace target
{
TrackFactory::TrackFactory(
    std::shared_ptr<const ITrackingAlgorithm> trackingAlgorithm
) : _trackingAlgorithm(std::move(trackingAlgorithm))
{
    if (!_trackingAlgorithm)
    {
        throw std::invalid_argument("Tracking algorithm must not be null");
    }
}

TentativeTrack TrackFactory::from(std::uint32_t id, Observation observation) const
{
    return TentativeTrack(id, std::move(observation), _trackingAlgorithm);
}

std::optional<ConfirmedTrack> TrackFactory::from(
    const TentativeTrack& tentative,
    const Observation& observation
) {
    return tentative.confirm(observation);
}

} // namespace target
