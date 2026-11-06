#pragma once

#include "domain/tentative_track.hpp"

#include <list>
#include <variant>

namespace target
{

using RegisteredTarget = std::variant<TentativeTrack, Track>;

// Registration order and identity survive confirmation.
class TrackRepository
{
public:
    std::uint32_t add(TentativeTrack track);
    std::uint32_t add(Track track);
    TentativeTrack* findTentative(std::uint32_t id);
    const TentativeTrack* findTentative(std::uint32_t id) const;
    Track* findTrack(std::uint32_t id);
    const Track* findTrack(std::uint32_t id) const;
    bool replace(Track track);
    bool remove(std::uint32_t id);
    const std::list<RegisteredTarget>& targets() const;

private:
    std::uint32_t add(RegisteredTarget track);

    std::list<RegisteredTarget> _targets;
};

} // namespace target
