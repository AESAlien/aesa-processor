#include "math/angle.hpp"

#include <cmath>
#include <stdexcept>

namespace math
{

const double PI = 3.14159265358979323846;

Angle::Angle(double degrees) noexcept : _degrees(degrees)
{
}

Angle Angle::fromDegrees(double degrees) noexcept
{
    return Angle(degrees);
}

Angle Angle::fromRadians(double radians) noexcept
{
    return Angle(radians * (180.0 / PI));
}

double Angle::deg() const noexcept
{
    return _degrees;
}

double Angle::rad() const noexcept
{
    return _degrees * (PI / 180.0);
}

Angle Angle::wrap180() const noexcept
{
    double degrees = std::fmod(_degrees + 180.0, 360.0);
    if (degrees < 0.0)
    {
        degrees += 360.0;
    }
    return Angle(degrees - 180.0);
}

Angle Angle::operator+() const noexcept
{
    return *this;
}

Angle Angle::operator-() const noexcept
{
    return Angle(-_degrees);
}

Angle Angle::operator+(const Angle& other) const noexcept
{
    Angle result(*this);
    return result += other;
}

Angle Angle::operator-(const Angle& other) const noexcept
{
    Angle result(*this);
    return result -= other;
}

Angle Angle::operator*(double scalar) const noexcept
{
    Angle result(*this);
    return result *= scalar;
}

Angle Angle::operator/(double scalar) const
{
    Angle result(*this);
    return result /= scalar;
}

Angle& Angle::operator+=(const Angle& other) noexcept
{
    _degrees += other._degrees;
    return *this;
}

Angle& Angle::operator-=(const Angle& other) noexcept
{
    _degrees -= other._degrees;
    return *this;
}

Angle& Angle::operator*=(double scalar) noexcept
{
    _degrees *= scalar;
    return *this;
}

Angle& Angle::operator/=(double scalar)
{
    if (scalar == 0.0)
    {
        throw std::domain_error("Cannot divide an angle by zero");
    }
    _degrees /= scalar;
    return *this;
}

bool Angle::operator==(const Angle& other) const noexcept
{
    return _degrees == other._degrees;
}

bool Angle::operator!=(const Angle& other) const noexcept
{
    return _degrees != other._degrees;
}

bool Angle::operator<(const Angle& other) const noexcept
{
    return _degrees < other._degrees;
}

bool Angle::operator<=(const Angle& other) const noexcept
{
    return _degrees <= other._degrees;
}

bool Angle::operator>(const Angle& other) const noexcept
{
    return _degrees > other._degrees;
}

bool Angle::operator>=(const Angle& other) const noexcept
{
    return _degrees >= other._degrees;
}

Angle operator*(double scalar, const Angle& angle) noexcept
{
    return angle * scalar;
}

namespace literals
{

Angle operator""_deg(unsigned long long degrees) noexcept
{
    return Angle::fromDegrees(static_cast<double>(degrees));
}

Angle operator""_deg(long double degrees) noexcept
{
    return Angle::fromDegrees(static_cast<double>(degrees));
}

} // namespace literals

} // namespace math
