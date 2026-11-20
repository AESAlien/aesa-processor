#pragma once

#include <protocol/ST_MsgHeader.hpp>

namespace comm::protocol
{

#pragma pack(push, 1)
// NOLINTBEGIN(readability-identifier-naming)
struct ST_RadarAttMsg
{
    ST_MsgHeader msgHeader;
    double radar_lat_deg;
    double radar_lon_deg;
    double radar_alt_km;
    double roll_deg;
    double pitch_deg;
    double yaw_deg;
};
// NOLINTEND(readability-identifier-naming)
#pragma pack(pop)

static_assert(sizeof(ST_RadarAttMsg) == 68, "ST_RadarAttMsg must be 68 bytes");

} // namespace comm::protocol
