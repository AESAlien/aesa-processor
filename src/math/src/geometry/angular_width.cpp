#include "math/geometry/angular_width.hpp"

#include <stdexcept>

namespace math
{

AngularWidth::AngularWidth(Angle angle)
    : _angle(angle)
{
    validate();
}

void AngularWidth::validate() const
{
    if (_angle.deg() <= 0.0)
    {
        throw std::invalid_argument("AngularWidth must be greater than 0 degrees");
    }
}

double AngularWidth::deg() const noexcept
{
    return _angle.deg();
}

double AngularWidth::rad() const noexcept
{
    return _angle.rad();
}

bool AngularWidth::equals(
    const AngularWidth& other,
    const QuantityTolerance<Angle>& tolerance
) const noexcept {
    return _angle.equals(other._angle, tolerance);
}

bool AngularWidth::equals(
    Angle other,
    const QuantityTolerance<Angle>& tolerance
) const noexcept {
    return _angle.equals(other, tolerance);
}

AngularWidth AngularWidth::operator+(Angle offset) const
{
    return AngularWidth(_angle + offset);
}

AngularWidth AngularWidth::operator-(Angle offset) const
{
    return AngularWidth(_angle - offset);
}

AngularWidth& AngularWidth::operator+=(Angle offset)
{
    return *this = *this + offset;
}

AngularWidth& AngularWidth::operator-=(Angle offset)
{
    return *this = *this - offset;
}

bool AngularWidth::operator==(const AngularWidth& other) const noexcept
{
    return equals(other);
}

bool AngularWidth::operator!=(const AngularWidth& other) const noexcept
{
    return !(*this == other);
}

bool AngularWidth::operator==(Angle other) const noexcept
{
    return equals(other);
}

bool AngularWidth::operator!=(Angle other) const noexcept
{
    return !(*this == other);
}

AngularWidth operator+(Angle offset, const AngularWidth& width)
{
    return width + offset;
}

bool operator==(Angle angle, const AngularWidth& width) noexcept
{
    return width == angle;
}

bool operator!=(Angle angle, const AngularWidth& width) noexcept
{
    return !(angle == width);
}

}
