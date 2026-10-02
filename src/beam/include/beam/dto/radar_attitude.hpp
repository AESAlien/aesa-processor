#pragma once
#include <cstdint>

namespace beam
{

struct RadarAttitude
{
    double  radar_lat_deg   {};
    double  radar_lon_deg   {};
    double  radar_alt_km    {};
    double  roll_deg        {};
    double  pitch_deg       {};
    double  yaw_deg         {};
};

static_assert(sizeof(RadarAttitude) == 48, "RadarAttitude must be 48 bytes");

}   // namespace beam
