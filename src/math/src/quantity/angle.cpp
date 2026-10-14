#include "math/quantity/angle.hpp"

#include <cmath>
#include <stdexcept>

namespace math
{
using namespace literals;

const QuantityTolerance<Angle> Angle::DEFAULT_TOLERANCE{
    Angle::fromDegrees(DEFAULT_ABSOLUTE_TOLERANCE),
    DEFAULT_RELATIVE_TOLERANCE
};

const double PI = 3.14159265358979323846;

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
    return _value;
}

double Angle::rad() const noexcept
{
    return _value * (PI / 180.0);
}

Angle Angle::wrap180() const noexcept
{
    // 먼저 360도 주기의 나머지를 구해 큰 각도에서 오프셋 정밀도 손실을 피한다.
    double degrees = std::fmod(_value, 360.0);
    if (degrees < 0.0)
    {
        degrees += 360.0;
    }

    if (degrees >= 180.0)
    {
        degrees -= 360.0;
    }

    return Angle(degrees);
}

Angle Angle::wrap90() const noexcept
{
    // 먼저 180도 주기의 나머지를 구해 큰 각도에서 오프셋 정밀도 손실을 피한다.
    double degrees = std::fmod(_value, 180.0);
    if (degrees < 0.0)
    {
        degrees += 180.0;
    }

    if (degrees >= 90.0)
    {
        degrees -= 180.0;
    }

    return Angle(degrees);
}

Angle Angle::operator+() const noexcept
{
    return *this;
}

Angle Angle::operator-() const noexcept
{
    return Angle(-_value);
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
    _value += other._value;

    return *this;
}

Angle& Angle::operator-=(const Angle& other) noexcept
{
    _value -= other._value;

    return *this;
}

Angle& Angle::operator*=(double scalar) noexcept
{
    _value *= scalar;

    return *this;
}

Angle& Angle::operator/=(double scalar)
{
    if (scalar == 0.0)
    {
        throw std::domain_error("Cannot divide an angle by zero");
    }

    _value /= scalar;

    return *this;
}

Angle::Angle(double degrees) noexcept
    : Quantity<Angle>(degrees)
{
}

Angle abs(Angle angle) noexcept
{
    return Angle::fromDegrees(std::abs(angle.deg()));
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

Angle operator""_rad(unsigned long long radians) noexcept
{
    return Angle::fromRadians(static_cast<double>(radians));
}

Angle operator""_rad(long double radians) noexcept
{
    return Angle::fromRadians(static_cast<double>(radians));
}

}

}
