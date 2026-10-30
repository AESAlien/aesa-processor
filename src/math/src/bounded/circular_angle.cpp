#include "math/bounded/circular_angle.hpp"

namespace math
{

CircularAngle::CircularAngle(Angle angle) noexcept
    : _angle(wrap360(angle))
{
}

CircularAngle CircularAngle::fromDegrees(double degrees)
{
    return CircularAngle(Angle::fromDegrees(degrees));
}

CircularAngle CircularAngle::fromRadians(double radians)
{
    return CircularAngle(Angle::fromRadians(radians));
}

double CircularAngle::deg() const noexcept
{
    return _angle.deg();
}

double CircularAngle::rad() const noexcept
{
    return _angle.rad();
}

bool CircularAngle::isEquivalentTo(
    const CircularAngle& other,
    const QuantityTolerance<Angle>& tolerance
) const noexcept {
    return _angle.isEquivalentTo(other._angle, tolerance);
}

bool CircularAngle::isEquivalentTo(
    Angle other,
    const QuantityTolerance<Angle>& tolerance
) const noexcept {
    return _angle.isEquivalentTo(other, tolerance);
}

CircularAngle CircularAngle::operator+(Angle offset) const
{
    return CircularAngle(_angle + offset);
}

CircularAngle CircularAngle::operator-(Angle offset) const
{
    return CircularAngle(_angle - offset);
}

CircularAngle& CircularAngle::operator+=(Angle offset)
{
    return *this = *this + offset;
}

CircularAngle& CircularAngle::operator-=(Angle offset)
{
    return *this = *this - offset;
}

bool CircularAngle::operator==(const CircularAngle& other) const noexcept
{
    return isEquivalentTo(other);
}

bool CircularAngle::operator!=(const CircularAngle& other) const noexcept
{
    return !(*this == other);
}

bool CircularAngle::operator==(Angle other) const noexcept
{
    return isEquivalentTo(other);
}

bool CircularAngle::operator!=(Angle other) const noexcept
{
    return !(*this == other);
}

CircularAngle operator+(Angle offset, const CircularAngle& azimuth)
{
    return azimuth + offset;
}

bool operator==(Angle angle, const CircularAngle& azimuth) noexcept
{
    return azimuth == angle;
}

bool operator!=(Angle angle, const CircularAngle& azimuth) noexcept
{
    return !(angle == azimuth);
}

}
