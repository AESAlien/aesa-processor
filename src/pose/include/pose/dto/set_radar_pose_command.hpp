#pragma once

#include <math/bounded/circular_angle.hpp>
#include <math/bounded/tilt_angle.hpp>
#include <math/quantity/angle.hpp>
#include <math/quantity/length.hpp>
#include <optional>

namespace pose
{

struct SetRadarPoseCommand
{
    math::TiltAngle latitude;
    math::CircularAngle longitude;
    math::Length altitude;
    math::Angle roll;
    math::TiltAngle pitch;
    math::Angle yaw;

    SetRadarPoseCommand(
        math::TiltAngle latitude,
        math::CircularAngle longitude,
        math::Length altitude,
        math::Angle roll,
        math::TiltAngle pitch,
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
        Builder& latitude(math::Angle value) { return latitude(math::TiltAngle(value)); }
        Builder& latitude(math::TiltAngle value) { _latitude = value; return *this; }
        Builder& longitude(math::Angle value) { return longitude(math::CircularAngle(value)); }
        Builder& longitude(math::CircularAngle value) { _longitude = value; return *this; }
        Builder& altitude(math::Length value) { _altitude = value; return *this; }
        Builder& roll(math::Angle value) { _roll = value; return *this; }
        Builder& pitch(math::Angle value) { return pitch(math::TiltAngle(value)); }
        Builder& pitch(math::TiltAngle value) { _pitch = value; return *this; }
        Builder& yaw(math::Angle value) { _yaw = value; return *this; }

        SetRadarPoseCommand build() const
        {
            return SetRadarPoseCommand(
                _latitude.value(), _longitude.value(), _altitude.value(),
                _roll.value(), _pitch.value(), _yaw.value());
        }

    private:
        std::optional<math::TiltAngle> _latitude;
        std::optional<math::CircularAngle> _longitude;
        std::optional<math::Length> _altitude;
        std::optional<math::Angle> _roll;
        std::optional<math::TiltAngle> _pitch;
        std::optional<math::Angle> _yaw;
    };
};

static_assert(sizeof(SetRadarPoseCommand) == 48, "SetRadarPoseCommand must be 48 bytes");

} // namespace pose
