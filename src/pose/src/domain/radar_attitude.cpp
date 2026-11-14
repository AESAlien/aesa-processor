#include <pose/domain/radar_attitude.hpp>

namespace pose
{

RadarAttitude::RadarAttitude(
    math::CircularAngle roll,
    math::TiltAngle pitch,
    math::CircularAngle yaw
) : _roll(roll),
    _pitch(pitch),
    _yaw(yaw)
{
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

} // namespace pose
