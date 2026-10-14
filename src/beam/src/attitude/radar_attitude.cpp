#include "attitude/radar_attitude.hpp"
#include "attitude/attitude_error.hpp"

namespace beam
{
namespace
{
using namespace math::literals;

void validate(
    math::Angle latitude,
    math::Angle longitude,
    math::Distance altitude,
    math::Angle roll,
    math::Angle pitch,
    math::Angle yaw
) {
    // NaN은 크기 비교가 모두 false이므로 유한성을 먼저 확인한다.
    if (!latitude.isFinite() || !longitude.isFinite() || !altitude.isFinite() ||
        !roll.isFinite() || !pitch.isFinite() || !yaw.isFinite())
    {
        throw AttitudeError(AttitudeErrorCode::NOT_FINITE, "RadarAttitude contains NaN or Inf");
    }

    if (latitude  < -90_deg  || latitude  > 90_deg ||
        longitude < -180_deg || longitude > 180_deg ||
        altitude  < -0.5_km  || altitude  > 100_km ||
        roll      < -180_deg || roll      > 180_deg ||
        yaw       < -360_deg || yaw       > 360_deg
    ) {
        throw AttitudeError(AttitudeErrorCode::OUT_OF_RANGE, "RadarAttitude value is out of range");
    }
}
} // namespace

RadarAttitude::RadarAttitude(
    math::Angle latitude,
    math::Angle longitude,
    math::Distance altitude,
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
    validate(_latitude, _longitude, _altitude, _roll, _pitch, _yaw);
}

math::Angle RadarAttitude::latitude() const noexcept
{
    return _latitude;
}

math::Angle RadarAttitude::longitude() const noexcept
{
    return _longitude;
}

math::Distance RadarAttitude::altitude() const noexcept
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

} // namespace beam
