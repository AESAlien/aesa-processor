#pragma once

#include <math/angle.hpp>
#include <math/distance.hpp>

namespace beam
{

struct SetAttitudeCommand
{
    SetAttitudeCommand(
        math::Angle latitude,
        math::Angle longitude,
        math::Distance altitude,
        math::Angle roll,
        math::Angle pitch,
        math::Angle yaw
    ) : latitude(latitude),
        longitude(longitude),
        altitude(altitude),
        roll(roll),
        pitch(pitch),
        yaw(yaw)
    {
    }

    const math::Angle latitude;
    const math::Angle longitude;
    const math::Distance altitude;
    const math::Angle roll;
    const math::Angle pitch;
    const math::Angle yaw;
};

static_assert(sizeof(SetAttitudeCommand) == 48, "SetAttitudeCommand must be 48 bytes");

} // namespace beam
