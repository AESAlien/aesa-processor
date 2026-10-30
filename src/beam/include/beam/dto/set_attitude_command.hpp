#pragma once

#include <math/bounded/circular_angle.hpp>
#include <math/bounded/tilt_angle.hpp>
#include <math/quantity/angle.hpp>
#include <math/quantity/length.hpp>
#include <optional>

namespace beam
{

struct SetAttitudeCommand
{
    math::TiltAngle latitude;
    math::CircularAngle longitude;
    math::Length altitude;
    math::Angle roll;
    math::Angle pitch;
    math::Angle yaw;

    SetAttitudeCommand(
        math::Angle latitude,
        math::Angle longitude,
        math::Length altitude,
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

    class Builder
    {
    public:
        Builder& latitude(math::Angle value) { _latitude = value; return *this; }
        Builder& longitude(math::Angle value) { _longitude = value; return *this; }
        Builder& altitude(math::Length value) { _altitude = value; return *this; }
        Builder& roll(math::Angle value) { _roll = value; return *this; }
        Builder& pitch(math::Angle value) { _pitch = value; return *this; }
        Builder& yaw(math::Angle value) { _yaw = value; return *this; }

        SetAttitudeCommand build() const
        {
            return SetAttitudeCommand(
                _latitude.value(), _longitude.value(), _altitude.value(),
                _roll.value(), _pitch.value(), _yaw.value());
        }

    private:
        std::optional<math::Angle> _latitude;
        std::optional<math::Angle> _longitude;
        std::optional<math::Length> _altitude;
        std::optional<math::Angle> _roll;
        std::optional<math::Angle> _pitch;
        std::optional<math::Angle> _yaw;
    };
};

static_assert(sizeof(SetAttitudeCommand) == 48, "SetAttitudeCommand must be 48 bytes");

} // namespace beam
