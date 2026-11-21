#include "track_factory.hpp"

#include <limits>
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

TentativeTrack TrackFactory::createTentative(Observation observation)
{
    if (_nextId > std::numeric_limits<std::uint16_t>::max())
    {
        throw std::overflow_error("Track ID exceeds the console ID range");
    }
    auto track = TentativeTrack(_nextId, std::move(observation), _trackingAlgorithm);
    ++_nextId;
    return track;
}

} // namespace target
