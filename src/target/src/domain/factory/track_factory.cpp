#include "track_factory.hpp"

#include <limits>
#include <stdexcept>
#include <utility>

namespace target
{
namespace
{

constexpr std::uint32_t MAX_TRACK_ID = std::numeric_limits<std::uint16_t>::max();

} // namespace

TrackFactory::TrackFactory(
    std::shared_ptr<const ITrackingAlgorithm> trackingAlgorithm,
    TrackLifecycleConfig lifecycleConfig
) : _trackingAlgorithm(std::move(trackingAlgorithm)),
    _lifecycleConfig(std::move(lifecycleConfig))
{
    if (!_trackingAlgorithm)
    {
        throw std::invalid_argument("Tracking algorithm must not be null");
    }
}

TentativeTrack TrackFactory::createTentative(Observation observation)
{
    if (_nextId > MAX_TRACK_ID)
    {
        throw std::overflow_error("Track ID exceeds the console ID range");
    }
    auto track = TentativeTrack(_nextId, std::move(observation), _lifecycleConfig, _trackingAlgorithm);
    ++_nextId;
    return track;
}

} // namespace target
