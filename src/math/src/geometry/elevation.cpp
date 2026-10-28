#include "math/geometry/elevation.hpp"

#include <stdexcept>

namespace math
{

Elevation::Elevation(Angle angle)
    : _angle(angle)
{
    validate();
}

void Elevation::validate() const
{
    if (_angle.deg() < -90.0 || _angle.deg() > 90.0)
    {
        throw std::invalid_argument("Elevation must be in [-90, 90] degrees");
    }
}

Elevation Elevation::fromDegrees(double degrees)
{
    return Elevation(Angle::fromDegrees(degrees));
}

Elevation Elevation::fromRadians(double radians)
{
    return Elevation(Angle::fromRadians(radians));
}

double Elevation::deg() const noexcept
{
    return _angle.deg();
}

double Elevation::rad() const noexcept
{
    return _angle.rad();
}

bool Elevation::equals(
    const Elevation& other,
    const QuantityTolerance<Angle>& tolerance
) const noexcept {
    return _angle.equals(other._angle, tolerance);
}

bool Elevation::equals(
    Angle other,
    const QuantityTolerance<Angle>& tolerance
) const noexcept {
    return _angle.equals(other, tolerance);
}

Elevation Elevation::operator+(Angle offset) const
{
    return Elevation(_angle + offset);
}

Elevation Elevation::operator-(Angle offset) const
{
    return Elevation(_angle - offset);
}

Elevation& Elevation::operator+=(Angle offset)
{
    return *this = *this + offset;
}

Elevation& Elevation::operator-=(Angle offset)
{
    return *this = *this - offset;
}

bool Elevation::operator==(const Elevation& other) const noexcept
{
    return equals(other);
}

bool Elevation::operator!=(const Elevation& other) const noexcept
{
    return !(*this == other);
}

bool Elevation::operator==(Angle other) const noexcept
{
    return equals(other);
}

bool Elevation::operator!=(Angle other) const noexcept
{
    return !(*this == other);
}

Elevation operator+(Angle offset, const Elevation& elevation)
{
    return elevation + offset;
}

bool operator==(Angle angle, const Elevation& elevation) noexcept
{
    return elevation == angle;
}

bool operator!=(Angle angle, const Elevation& elevation) noexcept
{
    return !(angle == elevation);
}

}
