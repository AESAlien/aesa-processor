#pragma once

namespace beam
{

struct SetAttitudeCommand
{
    double latitude_deg{};
    double longitude_deg{};
    double altitude_km{};
    double roll_deg{};
    double pitch_deg{};
    double yaw_deg{};
};

static_assert(sizeof(SetAttitudeCommand) == 48, "SetAttitudeCommand must be 48 bytes");

} // namespace beam
