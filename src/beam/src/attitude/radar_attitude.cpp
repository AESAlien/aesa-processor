#include "attitude/radar_attitude.hpp"

#include <stdexcept>

namespace beam
{

RadarAttitude::RadarAttitude(
    math::Angle latitude,
    math::Angle longitude,
    math::Length altitude,
    math::Angle roll,
    math::Angle pitch,
    math::Angle yaw
) : _latitude(latitude),
    _longitude(longitude),
    _altitude(altitude),
    _roll(roll),
    _pitch(pitch),
    _yaw(yaw)
{
    validate();
}

math::Angle RadarAttitude::latitude() const noexcept
{
    return _latitude;
}

math::Angle RadarAttitude::longitude() const noexcept
{
    return _longitude;
}

math::Length RadarAttitude::altitude() const noexcept
{
    return _altitude;
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

    if (_latitude  < -90_deg  || _latitude  > 90_deg ||
        _longitude < -180_deg || _longitude > 180_deg ||
        _altitude  < -0.5_km  || _altitude  > 100_km ||
        _roll      < -180_deg || _roll      > 180_deg ||
        _yaw       < -360_deg || _yaw       > 360_deg
    ) {
        throw std::invalid_argument("RadarAttitude: value is out of range");
    }
}

} // namespace beam
