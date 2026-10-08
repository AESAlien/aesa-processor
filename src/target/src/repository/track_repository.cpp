#include "track_repository.hpp"

#include <algorithm>

namespace target
{

void TrackRepository::upsert(TrackEvent trackEvent)
{
    const auto existing = std::find_if(_tracks.begin(), _tracks.end(),
        [&trackEvent](const TrackEvent& track)
        {
            return track.id == trackEvent.id;
        });
    if (existing != _tracks.end())
    {
        *existing = trackEvent;
        return;
    }
    _tracks.push_back(trackEvent);
}

bool TrackRepository::remove(std::uint16_t id)
{
    const auto existing = std::find_if(_tracks.begin(), _tracks.end(),
        [id](const TrackEvent& track)
        {
            return track.id == id;
        });
    if (existing == _tracks.end())
    {
        return false;
    }
    _tracks.erase(existing);
    return true;
}

const std::vector<TrackEvent>& TrackRepository::tracks() const
{
    return _tracks;
}

} // namespace target
