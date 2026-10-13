#include "math/quantity/angle.hpp"

#include <cmath>
#include <stdexcept>

namespace math
{
using namespace literals;

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
    return _degrees;
}

double Angle::rad() const noexcept
{
    return _degrees * (PI / 180.0);
}

bool Angle::isFinite() const noexcept
{
    return std::isfinite(_degrees);
}

bool Angle::isNonnegative() const noexcept
{
    return _degrees >= 0.0;
}

Angle Angle::wrap180() const noexcept
{
    // 먼저 360도 주기의 나머지를 구해 큰 각도에서 오프셋 정밀도 손실을 피한다.
    double degrees = std::fmod(_degrees, 360.0);
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
    double degrees = std::fmod(_degrees, 180.0);
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

bool Angle::equals(const Angle& other, const Tolerance<Angle>& tolerance) const noexcept
{
    const Tolerance<double> numericTolerance{tolerance.absolute()._degrees, tolerance.relative()};
    return numericTolerance.isWithin(_degrees, other._degrees);
}

bool Angle::operator!=(const Angle& other) const noexcept
{
    return !(*this == other);
}

bool Angle::operator==(const Angle& other) const noexcept
{
    // 타입별로 한 번만 초기화해 모든 기본 비교에서 공유한다.
    static const Tolerance<Angle> DEFAULT_ANGLE_TOLERANCE{
        fromDegrees(DEFAULT_TOLERANCE.absolute()),
        DEFAULT_TOLERANCE.relative()
    };
    return equals(other, DEFAULT_ANGLE_TOLERANCE);
}

bool Angle::operator<(const Angle& other) const noexcept
{
    return _degrees < other._degrees && *this != other;
}

bool Angle::operator<=(const Angle& other) const noexcept
{
    return _degrees <= other._degrees || *this == other;
}

bool Angle::operator>(const Angle& other) const noexcept
{
    return _degrees > other._degrees && *this != other;
}

bool Angle::operator>=(const Angle& other) const noexcept
{
    return _degrees >= other._degrees || *this == other;
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

Angle::Angle(double degrees) noexcept
    : _degrees(degrees)
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
