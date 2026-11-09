#include "track_repository.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace target
{
namespace
{

constexpr std::uint32_t MAX_TRACK_ID = std::numeric_limits<std::uint16_t>::max();

std::uint32_t targetId(const Track& target)
{
    return std::visit(
        [](const auto& value)
        {
            return value.id();
        },
        target
    );
}

template <typename Targets>
auto findTarget(Targets& targets, std::uint32_t id)
{
    return std::find_if(
        targets.begin(),
        targets.end(),
        [id](const Track& target)
        {
            return targetId(target) == id;
        }
    );
}

} // namespace

std::uint32_t TrackRepository::nextId() const
{
    if (_nextId > MAX_TRACK_ID)
    {
        throw std::overflow_error("Target ID exceeds the console ID range");
    }
    return _nextId;
}

std::uint32_t TrackRepository::add(TentativeTrack track)
{
    return add(Track(std::move(track)));
}

std::uint32_t TrackRepository::add(ConfirmedTrack track)
{
    return add(Track(std::move(track)));
}

std::uint32_t TrackRepository::add(Track track)
{
    const auto id = targetId(track);
    if (id > MAX_TRACK_ID)
    {
        throw std::overflow_error("Target ID exceeds the console ID range");
    }
    if (findTarget(_targets, id) != _targets.end())
    {
        throw std::invalid_argument("Track ID is already registered");
    }
    _targets.emplace_back(std::move(track));
    _nextId = std::max(_nextId, id + 1);
    return id;
}

TentativeTrack* TrackRepository::findTentative(std::uint32_t id)
{
    const auto target = findTarget(_targets, id);
    return target != _targets.end() ? std::get_if<TentativeTrack>(&*target) : nullptr;
}

const TentativeTrack* TrackRepository::findTentative(std::uint32_t id) const
{
    const auto target = findTarget(_targets, id);
    return target != _targets.end() ? std::get_if<TentativeTrack>(&*target) : nullptr;
}

ConfirmedTrack* TrackRepository::findTrack(std::uint32_t id)
{
    const auto target = findTarget(_targets, id);
    return target != _targets.end() ? std::get_if<ConfirmedTrack>(&*target) : nullptr;
}

const ConfirmedTrack* TrackRepository::findTrack(std::uint32_t id) const
{
    const auto target = findTarget(_targets, id);
    return target != _targets.end() ? std::get_if<ConfirmedTrack>(&*target) : nullptr;
}

bool TrackRepository::replace(ConfirmedTrack track)
{
    const auto target = findTarget(_targets, track.id());
    if (target == _targets.end())
    {
        return false;
    }
    // Replacement preserves the list node and cannot fail after destroying its old value.
    static_assert(std::is_nothrow_move_constructible_v<ConfirmedTrack>);
    target->emplace<ConfirmedTrack>(std::move(track));
    return true;
}

bool TrackRepository::remove(std::uint32_t id)
{
    const auto target = findTarget(_targets, id);
    if (target == _targets.end())
    {
        return false;
    }
    _targets.erase(target);
    return true;
}

const std::list<Track>& TrackRepository::targets() const
{
    return _targets;
}

} // namespace target
