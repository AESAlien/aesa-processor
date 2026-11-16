#pragma once

#include <pose/domain/geo_position.hpp>
#include <target/dto/track_event.hpp>

namespace target
{

class ConfirmedTrack;

class TrackEventFactory
{
public:
    static TrackEvent from(const ConfirmedTrack& confirmedTrack, const pose::GeoPosition& geodeticPosition);
};

} // namespace target
