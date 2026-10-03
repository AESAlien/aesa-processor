#pragma once

#include <math/angle.hpp>
#include <math/distance.hpp>

namespace beam
{

using math::literals::operator""_deg;
using math::literals::operator""_km;

struct SetAttitudeCommand
{
    math::Angle latitude = 0_deg;
    math::Angle longitude = 0_deg;
    math::Distance altitude = 0_km;
    math::Angle roll = 0_deg;
    math::Angle pitch = 0_deg;
    math::Angle yaw = 0_deg;
};

static_assert(sizeof(SetAttitudeCommand) == 48, "SetAttitudeCommand must be 48 bytes");

} // namespace beam
