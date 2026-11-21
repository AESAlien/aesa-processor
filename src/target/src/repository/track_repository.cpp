#include "track_repository.hpp"

#include <algorithm>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace target
{
namespace
{

template <typename Tracks>
auto findById(Tracks& tracks, std::uint32_t id)
{
    return std::find_if(
        tracks.begin(),
        tracks.end(),
        [id](const Track& track)
        {
            return std::visit(
                [id](const auto& value)
                {
                    return value.id() == id;
                },
                track
            );
        }
    );
}

template <typename TrackType>
std::vector<const TrackType*> tracksOfType(const std::list<Track>& tracks, std::size_t limit)
{
    std::vector<const TrackType*> matches;
    for (const auto& track : tracks)
    {
        if (matches.size() == limit)
        {
            break;
        }
        if (const auto* match = std::get_if<TrackType>(&track))
        {
            matches.push_back(match);
        }
    }
    return matches;
}

} // namespace

std::uint32_t TrackRepository::add(TentativeTrack tentativeTrack)
{
    return add(Track(std::move(tentativeTrack)));
}

std::uint32_t TrackRepository::add(ConfirmedTrack confirmedTrack)
{
    return add(Track(std::move(confirmedTrack)));
}

std::uint32_t TrackRepository::add(Track track)
{
    const auto id = std::visit(
        [](const auto& value)
        {
            return value.id();
        },
        track
    );
    if (findById(_tracks, id) != _tracks.end())
    {
        throw std::invalid_argument("Track ID is already registered");
    }
    _tracks.emplace_back(std::move(track));
    return id;
}

TentativeTrack* TrackRepository::findTentativeById(std::uint32_t id)
{
    const auto track = findById(_tracks, id);
    return track != _tracks.end() ? std::get_if<TentativeTrack>(&*track) : nullptr;
}

ConfirmedTrack* TrackRepository::findConfirmedById(std::uint32_t id)
{
    const auto track = findById(_tracks, id);
    return track != _tracks.end() ? std::get_if<ConfirmedTrack>(&*track) : nullptr;
}

bool TrackRepository::replace(ConfirmedTrack confirmedTrack)
{
    const auto track = findById(_tracks, confirmedTrack.id());
    if (track == _tracks.end())
    {
        return false;
    }
    // 기존 값을 파괴한 뒤에도 예외 없이 같은 목록 노드에 새 값을 생성해야 한다.
    static_assert(std::is_nothrow_move_constructible_v<ConfirmedTrack>);
    track->emplace<ConfirmedTrack>(std::move(confirmedTrack));
    return true;
}

bool TrackRepository::removeById(std::uint32_t id)
{
    const auto track = findById(_tracks, id);
    if (track == _tracks.end())
    {
        return false;
    }
    _tracks.erase(track);
    return true;
}

std::size_t TrackRepository::clear()
{
    const auto count = _tracks.size();
    _tracks.clear();
    return count;
}

const std::list<Track>& TrackRepository::tracks() const
{
    return _tracks;
}

std::vector<const ConfirmedTrack*> TrackRepository::confirmedTracks(std::size_t limit) const
{
    return tracksOfType<ConfirmedTrack>(_tracks, limit);
}

std::vector<const TentativeTrack*> TrackRepository::tentativeTracks(std::size_t limit) const
{
    return tracksOfType<TentativeTrack>(_tracks, limit);
}

} // namespace target
