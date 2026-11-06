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

std::optional<Track> TrackFactory::from(
    const TentativeTrack& tentative,
    const Observation& observation
) {
    return tentative.confirm(observation);
}

Track TrackFactory::from(
    std::uint32_t id,
    std::chrono::milliseconds initialTime,
    TargetTrackingState state,
    std::shared_ptr<const ITrackingAlgorithm> trackingAlgorithm
) {
    return Track(id, initialTime, std::move(state), std::move(trackingAlgorithm));
}

} // namespace target
