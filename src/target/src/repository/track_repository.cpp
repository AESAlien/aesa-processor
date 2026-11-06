#include "track_repository.hpp"

#include <algorithm>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace target
{
namespace
{

std::uint32_t targetId(const RegisteredTarget& target)
{
    return std::visit([](const auto& value) { return value.id(); }, target);
}

template <typename Targets>
auto findTarget(Targets& targets, std::uint32_t id)
{
    return std::find_if(targets.begin(), targets.end(),
        [id](const RegisteredTarget& target)
        {
            return targetId(target) == id;
        });
}

} // namespace

std::uint32_t TrackRepository::add(TentativeTrack track)
{
    return add(RegisteredTarget(std::move(track)));
}

std::uint32_t TrackRepository::add(Track track)
{
    return add(RegisteredTarget(std::move(track)));
}

std::uint32_t TrackRepository::add(RegisteredTarget track)
{
    const auto id = targetId(track);
    if (findTarget(_targets, id) != _targets.end())
    {
        throw std::invalid_argument("Track ID is already registered");
    }
    _targets.emplace_back(std::move(track));
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

Track* TrackRepository::findTrack(std::uint32_t id)
{
    const auto target = findTarget(_targets, id);
    return target != _targets.end() ? std::get_if<Track>(&*target) : nullptr;
}

const Track* TrackRepository::findTrack(std::uint32_t id) const
{
    const auto target = findTarget(_targets, id);
    return target != _targets.end() ? std::get_if<Track>(&*target) : nullptr;
}

bool TrackRepository::replace(Track track)
{
    const auto target = findTarget(_targets, track.id());
    if (target == _targets.end())
    {
        return false;
    }
    // Replacement preserves the list node and cannot fail after destroying its old value.
    static_assert(std::is_nothrow_move_constructible_v<Track>);
    target->emplace<Track>(std::move(track));
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

const std::list<RegisteredTarget>& TrackRepository::targets() const
{
    return _targets;
}

} // namespace target
