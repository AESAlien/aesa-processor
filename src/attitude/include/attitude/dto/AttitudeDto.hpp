#pragma once
#include <cstdint>

namespace dto
{

struct AttitudeDto
{
    double  radar_lat_deg;
    double  radar_lon_deg;
    double  radar_alt_km;
    double  roll_deg;
    double  pitch_deg;
    double  yaw_deg;
};

static_assert(sizeof(AttitudeDto) == 48, "AttitudeDto must be 48 bytes");

}
