#pragma once

#include "math/quantity/quantity_tolerance.hpp"

namespace math
{

extern const double PI;

class Angle : public Quantity<Angle>
{
public:
    static const QuantityTolerance<Angle> DEFAULT_TOLERANCE;
    // 절대·상대 허용오차 없이 비교한다. 무한대·NaN 값은 같다고 판단하지 않는다.
    static const QuantityTolerance<Angle> NONE_TOLERANCE;

    static Angle fromDegrees(double degrees) noexcept;
    static Angle fromRadians(double radians) noexcept;

    double deg() const noexcept;
    double rad() const noexcept;

    // 원본을 유지하며 [-180, 180)도 범위로 정규화한 새 각도를 반환한다.
    Angle wrap180() const noexcept;

    // 원본을 유지하며 [-90, 90)도 범위로 정규화한 새 각도를 반환한다.
    Angle wrap90() const noexcept;

    Angle operator+() const noexcept;
    Angle operator-() const noexcept;

    Angle operator+(const Angle& other) const noexcept;
    Angle operator-(const Angle& other) const noexcept;

    Angle operator*(double scalar) const noexcept;
    // 0으로 나누면 std::domain_error를 던진다.
    Angle operator/(double scalar) const;

    Angle& operator+=(const Angle& other) noexcept;
    Angle& operator-=(const Angle& other) noexcept;
    Angle& operator*=(double scalar) noexcept;
    // 0으로 나누면 원본을 유지하고 std::domain_error를 던진다.
    Angle& operator/=(double scalar);

private:
    explicit Angle(double degrees) noexcept;
};

Angle abs(Angle angle) noexcept;
Angle operator*(double scalar, const Angle& angle) noexcept;

namespace literals
{

Angle operator""_deg(unsigned long long degrees) noexcept;
Angle operator""_deg(long double degrees) noexcept;
Angle operator""_rad(unsigned long long radians) noexcept;
Angle operator""_rad(long double radians) noexcept;

}

}
