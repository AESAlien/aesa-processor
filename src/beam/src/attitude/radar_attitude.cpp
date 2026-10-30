#include "attitude/radar_attitude.hpp"

#include <stdexcept>

namespace beam
{

RadarAttitude::RadarAttitude(
    math::Angle roll,
    math::TiltAngle pitch,
    math::Angle yaw
) : _roll(roll),
    _pitch(pitch),
    _yaw(yaw)
{
    validate(roll, yaw);
}

math::CircularAngle RadarAttitude::roll() const noexcept
{
    return _roll;
}

math::TiltAngle RadarAttitude::pitch() const noexcept
{
    return _pitch;
}

math::CircularAngle RadarAttitude::yaw() const noexcept
{
    return _yaw;
}

void RadarAttitude::validate(math::Angle roll, math::Angle yaw)
{
    using namespace math::literals;

    if (roll < -180_deg || roll > 180_deg)
    {
        throw std::invalid_argument("RadarAttitude: roll must be in [-180, 180] degrees");
    }

    if (yaw < -360_deg || yaw > 360_deg)
    {
        throw std::invalid_argument("RadarAttitude: yaw must be in [-360, 360] degrees");
    }
}

} // namespace beam
