#include "track_reporter.hpp"

#include "dto/factory/track_event_factory.hpp"
#include "repository/track_repository.hpp"

#include <stdexcept>
#include <utility>

namespace target
{

TrackReporter::TrackReporter(
    std::shared_ptr<const TrackRepository> repository,
    std::size_t maxReportedTrackCount
) : _trackRepository(std::move(repository)),
    _maxReportedTrackCount(maxReportedTrackCount)
{
    if (!_trackRepository)
    {
        throw std::invalid_argument("Track reporting requires a repository");
    }
}

std::vector<TrackEvent> TrackReporter::nextTrackEvents() const
{
    const auto confirmedTracks = _trackRepository->confirmedTracks(_maxReportedTrackCount);
    std::vector<TrackEvent> events;
    events.reserve(confirmedTracks.size());
    for (const auto* confirmedTrack : confirmedTracks)
    {
        events.push_back(TrackEventFactory::from(*confirmedTrack));
    }
    return events;
}

} // namespace target
