#pragma once

#include "math/quantity/quantity_tolerance.hpp"

namespace math
{

extern const double PI;

class Angle : public Quantity<Angle>
{
public:
    static const QuantityTolerance<Angle> DEFAULT_TOLERANCE;
    // 절대·상대 허용오차 없이 비교한다.
    static const QuantityTolerance<Angle> NONE_TOLERANCE;

    // Rejects non-finite stored values with std::invalid_argument.
    static Angle fromDegrees(double degrees);
    static Angle fromRadians(double radians);

    double deg() const noexcept;
    double rad() const noexcept;

    // 원본을 유지하며 [-180, 180)도 범위로 정규화한 새 각도를 반환한다.
    Angle wrap180() const noexcept;

    // 원본을 유지하며 [-90, 90)도 범위로 정규화한 새 각도를 반환한다.
    Angle wrap90() const noexcept;

    Angle operator+() const noexcept;
    Angle operator-() const noexcept;

    Angle operator+(const Angle& other) const;
    Angle operator-(const Angle& other) const;

    Angle operator*(double scalar) const;
    // 0으로 나누면 std::domain_error를 던진다.
    Angle operator/(double scalar) const;

    // Non-finite results throw std::invalid_argument without changing the value.
    Angle& operator+=(const Angle& other);
    Angle& operator-=(const Angle& other);
    Angle& operator*=(double scalar);
    // 0으로 나누면 원본을 유지하고 std::domain_error를 던진다.
    Angle& operator/=(double scalar);

private:
    explicit Angle(double degrees);
};

Angle abs(Angle angle) noexcept;
Angle operator*(double scalar, const Angle& angle);

namespace literals
{

Angle operator""_deg(unsigned long long degrees) noexcept;
Angle operator""_deg(long double degrees);
Angle operator""_rad(unsigned long long radians) noexcept;
Angle operator""_rad(long double radians);

}

}
