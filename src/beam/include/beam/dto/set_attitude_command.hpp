#pragma once

#include <math/quantity/angle.hpp>
#include <math/quantity/length.hpp>
#include <optional>

namespace aesa_processor::beam
{

struct SetAttitudeCommand
{
    const aesa_processor::math::Angle latitude;
    const aesa_processor::math::Angle longitude;
    const aesa_processor::math::Length altitude;
    const aesa_processor::math::Angle roll;
    const aesa_processor::math::Angle pitch;
    const aesa_processor::math::Angle yaw;

    SetAttitudeCommand(
        aesa_processor::math::Angle latitude,
        aesa_processor::math::Angle longitude,
        aesa_processor::math::Length altitude,
        aesa_processor::math::Angle roll,
        aesa_processor::math::Angle pitch,
        aesa_processor::math::Angle yaw
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
        Builder& latitude(aesa_processor::math::Angle value) { _latitude = value; return *this; }
        Builder& longitude(aesa_processor::math::Angle value) { _longitude = value; return *this; }
        Builder& altitude(aesa_processor::math::Length value) { _altitude = value; return *this; }
        Builder& roll(aesa_processor::math::Angle value) { _roll = value; return *this; }
        Builder& pitch(aesa_processor::math::Angle value) { _pitch = value; return *this; }
        Builder& yaw(aesa_processor::math::Angle value) { _yaw = value; return *this; }

        SetAttitudeCommand build() const
        {
            return SetAttitudeCommand(
                _latitude.value(), _longitude.value(), _altitude.value(),
                _roll.value(), _pitch.value(), _yaw.value());
        }

    private:
        std::optional<aesa_processor::math::Angle> _latitude;
        std::optional<aesa_processor::math::Angle> _longitude;
        std::optional<aesa_processor::math::Length> _altitude;
        std::optional<aesa_processor::math::Angle> _roll;
        std::optional<aesa_processor::math::Angle> _pitch;
        std::optional<aesa_processor::math::Angle> _yaw;
    };
};

static_assert(sizeof(SetAttitudeCommand) == 48, "SetAttitudeCommand must be 48 bytes");

} // namespace aesa_processor::beam
