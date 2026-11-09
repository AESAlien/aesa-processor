#pragma once

#include "domain/track.hpp"

#include <list>

namespace target
{

// Registration order and identity survive confirmation.
class TrackRepository
{
public:
    // The next identity is consumed only by a successful add().
    std::uint32_t nextId() const;
    std::uint32_t add(TentativeTrack track);
    std::uint32_t add(ConfirmedTrack track);
    TentativeTrack* findTentative(std::uint32_t id);
    const TentativeTrack* findTentative(std::uint32_t id) const;
    ConfirmedTrack* findTrack(std::uint32_t id);
    const ConfirmedTrack* findTrack(std::uint32_t id) const;
    bool replace(ConfirmedTrack track);
    bool remove(std::uint32_t id);
    const std::list<Track>& targets() const;

private:
    std::uint32_t add(Track track);

    std::list<Track> _targets;
    std::uint32_t _nextId = 1;
};

} // namespace target
