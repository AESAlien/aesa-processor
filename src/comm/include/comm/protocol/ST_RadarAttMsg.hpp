#pragma once
#include <cstdint>

namespace dto
{

struct ST_RadarAttMsg
{
    double  radar_lat_deg;
    double  radar_lon_deg;
    double  radar_alt_km;
    double  roll_deg;
    double  pitch_deg;
    double  yaw_deg;
};

static_assert(sizeof(ST_RadarAttMsg) == 48, "ST_RadarAttMsg must be 48 bytes");

}
