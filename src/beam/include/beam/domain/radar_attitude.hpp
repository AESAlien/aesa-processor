#pragma once
#include <cstdint>

namespace beam
{

struct RadarAttitude
{
    double radarLat_deg{};
    double radarLon_deg{};
    double radarAlt_km{};
    double roll_deg{};
    double pitch_deg{};
    double yaw_deg{};
};

static_assert(sizeof(RadarAttitude) == 48, "RadarAttitude must be 48 bytes");

} // namespace beam
