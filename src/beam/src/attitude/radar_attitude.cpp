#include "attitude/radar_attitude.hpp"
#include "attitude/attitude_error.hpp"

namespace aesa_processor::beam
{

RadarAttitude::RadarAttitude(
    aesa_processor::math::Angle latitude,
    aesa_processor::math::Angle longitude,
    aesa_processor::math::Length altitude,
    aesa_processor::math::Angle roll,
    aesa_processor::math::Angle pitch,
    aesa_processor::math::Angle yaw
) : _latitude(latitude),
    _longitude(longitude),
    _altitude(altitude),
    _roll(roll),
    _pitch(pitch),
    _yaw(yaw)
{
    validate();
}

aesa_processor::math::Angle RadarAttitude::latitude() const noexcept
{
    return _latitude;
}

aesa_processor::math::Angle RadarAttitude::longitude() const noexcept
{
    return _longitude;
}

aesa_processor::math::Length RadarAttitude::altitude() const noexcept
{
    return _altitude;
}

aesa_processor::math::Angle RadarAttitude::roll() const noexcept
{
    return _roll;
}

aesa_processor::math::Angle RadarAttitude::pitch() const noexcept
{
    return _pitch;
}

aesa_processor::math::Angle RadarAttitude::yaw() const noexcept
{
    return _yaw;
}

void RadarAttitude::validate() const
{
    using namespace aesa_processor::math::literals;

    if (_latitude  < -90_deg  || _latitude  > 90_deg ||
        _longitude < -180_deg || _longitude > 180_deg ||
        _altitude  < -0.5_km  || _altitude  > 100_km ||
        _roll      < -180_deg || _roll      > 180_deg ||
        _yaw       < -360_deg || _yaw       > 360_deg
    ) {
        throw AttitudeError(AttitudeErrorCode::OUT_OF_RANGE, "RadarAttitude value is out of range");
    }
}

} // namespace aesa_processor::beam
