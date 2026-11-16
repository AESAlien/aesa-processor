#include "track_reporter.hpp"

#include "dto/factory/track_event_factory.hpp"
#include "domain/confirmed_track.hpp"
#include "domain/track_state.hpp"
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

void TrackReporter::setEnuGeoTransform(pose::EnuGeoTransform enuGeoTransform)
{
    _enuGeoTransform = std::move(enuGeoTransform);
}

std::vector<TrackEvent> TrackReporter::getTrackEvents() const
{
    if (!_enuGeoTransform)
    {
        return {};
    }

    const auto confirmedTracks = _trackRepository->confirmedTracks(_maxReportedTrackCount);
    std::vector<TrackEvent> events;
    events.reserve(confirmedTracks.size());
    for (const auto* confirmedTrack : confirmedTracks)
    {
        const auto position = _enuGeoTransform->enuToGeo(confirmedTrack->state()->position());
        events.push_back(TrackEventFactory::from(*confirmedTrack, position));
    }
    return events;
}

} // namespace target
