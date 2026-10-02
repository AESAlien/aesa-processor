#pragma once
#include <cstdint>

namespace beam
{

struct RadarAttitude
{
    double latitude_deg{};
    double longitude_deg{};
    double altitude_km{};
    double roll_deg{};
    double pitch_deg{};
    double yaw_deg{};
};

static_assert(sizeof(RadarAttitude) == 48, "RadarAttitude must be 48 bytes");

} // namespace beam
