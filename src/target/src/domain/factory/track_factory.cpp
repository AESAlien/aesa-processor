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

TentativeTrack TrackFactory::createTentative(std::uint32_t id, Observation observation) const
{
    return TentativeTrack(id, std::move(observation), _trackingAlgorithm);
}

} // namespace target
