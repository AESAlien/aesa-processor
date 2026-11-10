#pragma once

#include <target/dto/track_event.hpp>

namespace target
{

class ConfirmedTrack;

class TrackEventFactory
{
public:
    static TrackEvent from(const ConfirmedTrack& confirmedTrack);
};

} // namespace target
