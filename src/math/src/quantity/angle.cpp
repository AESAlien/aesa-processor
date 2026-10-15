#include "math/quantity/angle.hpp"

#include <cmath>
#include <stdexcept>

namespace math
{
using namespace literals;

const QuantityTolerance<Angle> Angle::NONE_TOLERANCE{
    Angle::fromDegrees(0.0),
    0.0
};

const QuantityTolerance<Angle> Angle::DEFAULT_TOLERANCE{
    Angle::fromDegrees(DEFAULT_ABSOLUTE_TOLERANCE),
    DEFAULT_RELATIVE_TOLERANCE
};

const double PI = 3.14159265358979323846;

Angle Angle::fromDegrees(double degrees)
{
    return Angle(degrees);
}

Angle Angle::fromRadians(double radians)
{
    return Angle(radians * (180.0 / PI));
}

double Angle::deg() const noexcept
{
    return value();
}

double Angle::rad() const noexcept
{
    return value() * (PI / 180.0);
}

bool Angle::equals(
    const Angle& other,
    const QuantityTolerance<Angle>& tolerance
) const noexcept {
    if (tolerance.isWithin(*this, other))
    {
        return true;
    }

    return tolerance.isWithin(wrap180(*this), wrap180(other));
}

bool Angle::notEquals(
    const Angle& other,
    const QuantityTolerance<Angle>& tolerance
) const noexcept {
    return !equals(other, tolerance);
}

bool Angle::operator==(const Angle& other) const noexcept
{
    return equals(other);
}

bool Angle::operator!=(const Angle& other) const noexcept
{
    return notEquals(other);
}

Angle wrap180(Angle angle) noexcept
{
    // 먼저 360도 주기의 나머지를 구해 큰 각도에서 오프셋 정밀도 손실을 피한다.
    double degrees = std::fmod(angle.deg(), 360.0);
    if (degrees < 0.0)
    {
        degrees += 360.0;
    }

    if (degrees >= 180.0)
    {
        degrees -= 360.0;
    }

    return Angle::fromDegrees(degrees);
}

Angle wrap360(Angle angle) noexcept
{
    double degrees = std::fmod(angle.deg(), 360.0);
    if (degrees < 0.0)
    {
        degrees += 360.0;
    }

    if (degrees == 0.0)
    {
        degrees = 0.0;
    }

    return Angle::fromDegrees(degrees);
}

Angle Angle::operator+() const noexcept
{
    return *this;
}

Angle Angle::operator-() const noexcept
{
    return Angle(-value());
}

Angle Angle::operator+(const Angle& other) const
{
    Angle result(*this);

    return result += other;
}

Angle Angle::operator-(const Angle& other) const
{
    Angle result(*this);

    return result -= other;
}

Angle Angle::operator*(double scalar) const
{
    Angle result(*this);

    return result *= scalar;
}

Angle Angle::operator/(double scalar) const
{
    Angle result(*this);

    return result /= scalar;
}

Angle& Angle::operator+=(const Angle& other)
{
    *this = Angle(value() + other.value());

    return *this;
}

Angle& Angle::operator-=(const Angle& other)
{
    *this = Angle(value() - other.value());

    return *this;
}

Angle& Angle::operator*=(double scalar)
{
    *this = Angle(value() * scalar);

    return *this;
}

Angle& Angle::operator/=(double scalar)
{
    if (scalar == 0.0)
    {
        throw std::domain_error("Cannot divide an angle by zero");
    }

    *this = Angle(value() / scalar);

    return *this;
}

Angle::Angle(double degrees)
    : Quantity<Angle>(degrees)
{
}

Angle abs(Angle angle) noexcept
{
    return Angle::fromDegrees(std::abs(angle.deg()));
}

Angle operator*(double scalar, const Angle& angle)
{
    return angle * scalar;
}

namespace literals
{

Angle operator""_deg(unsigned long long degrees) noexcept
{
    return Angle::fromDegrees(static_cast<double>(degrees));
}

Angle operator""_deg(long double degrees)
{
    return Angle::fromDegrees(static_cast<double>(degrees));
}

Angle operator""_rad(unsigned long long radians) noexcept
{
    return Angle::fromRadians(static_cast<double>(radians));
}

Angle operator""_rad(long double radians)
{
    return Angle::fromRadians(static_cast<double>(radians));
}

}

}
