#include "attitude/radar_attitude.hpp"

#include <stdexcept>

namespace beam
{

RadarAttitude::RadarAttitude(
    math::Angle roll,
    math::Angle pitch,
    math::Angle yaw
) : _roll(roll),
    _pitch(pitch),
    _yaw(yaw)
{
    validate();
}

math::Angle RadarAttitude::roll() const noexcept
{
    return _roll;
}

math::Angle RadarAttitude::pitch() const noexcept
{
    return _pitch;
}

math::Angle RadarAttitude::yaw() const noexcept
{
    return _yaw;
}

void RadarAttitude::validate() const
{
    using namespace math::literals;

    if (_roll < -180_deg || _roll > 180_deg) {
        throw std::invalid_argument("RadarAttitude: roll must be in [-180, 180] degrees");
    }

    if (_yaw < -360_deg || _yaw > 360_deg) {
        throw std::invalid_argument("RadarAttitude: yaw must be in [-360, 360] degrees");
    }
}

} // namespace beam
