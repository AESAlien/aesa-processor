#include "math/geometry/tilt_angle.hpp"

#include "math/quantity/angle.hpp"

#include <stdexcept>

namespace math
{
using namespace literals;

TiltAngle::TiltAngle(Angle angle)
    : _angle(angle)
{
    validate();
}

void TiltAngle::validate() const
{
    if (_angle < -90.0_deg || _angle > 90.0_deg)
    {
        throw std::invalid_argument("TiltAngle must be in [-90, 90] degrees");
    }
}

TiltAngle TiltAngle::fromDegrees(double degrees)
{
    return TiltAngle(Angle::fromDegrees(degrees));
}

TiltAngle TiltAngle::fromRadians(double radians)
{
    return TiltAngle(Angle::fromRadians(radians));
}

double TiltAngle::deg() const noexcept
{
    return _angle.deg();
}

double TiltAngle::rad() const noexcept
{
    return _angle.rad();
}

bool TiltAngle::equals(
    const TiltAngle& other,
    const QuantityTolerance<Angle>& tolerance
) const noexcept {
    return _angle.equals(other._angle, tolerance);
}

bool TiltAngle::equals(
    Angle other,
    const QuantityTolerance<Angle>& tolerance
) const noexcept {
    return _angle.equals(other, tolerance);
}

TiltAngle TiltAngle::operator+(Angle offset) const
{
    return TiltAngle(_angle + offset);
}

TiltAngle TiltAngle::operator-(Angle offset) const
{
    return TiltAngle(_angle - offset);
}

TiltAngle& TiltAngle::operator+=(Angle offset)
{
    return *this = *this + offset;
}

TiltAngle& TiltAngle::operator-=(Angle offset)
{
    return *this = *this - offset;
}

bool TiltAngle::operator==(const TiltAngle& other) const noexcept
{
    return equals(other);
}

bool TiltAngle::operator!=(const TiltAngle& other) const noexcept
{
    return !(*this == other);
}

bool TiltAngle::operator==(Angle other) const noexcept
{
    return equals(other);
}

bool TiltAngle::operator!=(Angle other) const noexcept
{
    return !(*this == other);
}

TiltAngle operator+(Angle offset, const TiltAngle& elevation)
{
    return elevation + offset;
}

bool operator==(Angle angle, const TiltAngle& elevation) noexcept
{
    return elevation == angle;
}

bool operator!=(Angle angle, const TiltAngle& elevation) noexcept
{
    return !(angle == elevation);
}

}
