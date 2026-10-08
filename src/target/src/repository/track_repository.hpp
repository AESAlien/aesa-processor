#pragma once

#include <target/dto/track_event.hpp>

#include <cstdint>
#include <vector>

namespace target
{

// Current registered tracks, with one latest report per track ID.
class TrackRepository
{
public:
    void upsert(TrackEvent trackEvent);
    bool remove(std::uint16_t id);
    const std::vector<TrackEvent>& tracks() const;

private:
    std::vector<TrackEvent> _tracks;
};

} // namespace target
