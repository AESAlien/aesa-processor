#include "math/geometry/azimuth.hpp"

namespace math
{

Azimuth::Azimuth(Angle angle) noexcept
    : _angle(wrap360(angle))
{
}

Azimuth Azimuth::fromDegrees(double degrees)
{
    return Azimuth(Angle::fromDegrees(degrees));
}

Azimuth Azimuth::fromRadians(double radians)
{
    return Azimuth(Angle::fromRadians(radians));
}

double Azimuth::deg() const noexcept
{
    return _angle.deg();
}

double Azimuth::rad() const noexcept
{
    return _angle.rad();
}

bool Azimuth::isEquivalentTo(
    const Azimuth& other,
    const QuantityTolerance<Angle>& tolerance
) const noexcept {
    return _angle.isEquivalentTo(other._angle, tolerance);
}

bool Azimuth::isEquivalentTo(
    Angle other,
    const QuantityTolerance<Angle>& tolerance
) const noexcept {
    return _angle.isEquivalentTo(other, tolerance);
}

Azimuth Azimuth::operator+(Angle offset) const
{
    return Azimuth(_angle + offset);
}

Azimuth Azimuth::operator-(Angle offset) const
{
    return Azimuth(_angle - offset);
}

Azimuth& Azimuth::operator+=(Angle offset)
{
    return *this = *this + offset;
}

Azimuth& Azimuth::operator-=(Angle offset)
{
    return *this = *this - offset;
}

bool Azimuth::operator==(const Azimuth& other) const noexcept
{
    return isEquivalentTo(other);
}

bool Azimuth::operator!=(const Azimuth& other) const noexcept
{
    return !(*this == other);
}

bool Azimuth::operator==(Angle other) const noexcept
{
    return isEquivalentTo(other);
}

bool Azimuth::operator!=(Angle other) const noexcept
{
    return !(*this == other);
}

Azimuth operator+(Angle offset, const Azimuth& azimuth)
{
    return azimuth + offset;
}

bool operator==(Angle angle, const Azimuth& azimuth) noexcept
{
    return azimuth == angle;
}

bool operator!=(Angle angle, const Azimuth& azimuth) noexcept
{
    return !(angle == azimuth);
}

}
