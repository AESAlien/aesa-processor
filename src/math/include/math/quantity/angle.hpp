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

    // 저장할 값이 유한하지 않으면 std::invalid_argument를 던진다.
    static Angle fromDegrees(double degrees);
    // 저장할 값이 유한하지 않으면 std::invalid_argument를 던진다.
    static Angle fromRadians(double radians);

    double deg() const noexcept;
    double rad() const noexcept;

    // 360도 주기로 동치이고 지정한 허용오차 범위 안이면 true를 반환한다.
    bool isEquivalentTo(
        const Angle& other,
        const QuantityTolerance<Angle>& tolerance = DEFAULT_TOLERANCE
    ) const noexcept;

    Angle operator+() const noexcept;
    Angle operator-() const noexcept;

    Angle operator+(const Angle& other) const;
    Angle operator-(const Angle& other) const;

    Angle operator*(double scalar) const;
    // 0으로 나누면 std::domain_error를 던진다.
    Angle operator/(double scalar) const;

    // 결과가 유한하지 않으면 원본을 변경하지 않고 std::invalid_argument를 던진다.
    Angle& operator+=(const Angle& other);
    // 결과가 유한하지 않으면 원본을 변경하지 않고 std::invalid_argument를 던진다.
    Angle& operator-=(const Angle& other);
    // 결과가 유한하지 않으면 원본을 변경하지 않고 std::invalid_argument를 던진다.
    Angle& operator*=(double scalar);
    // 0으로 나누면 원본을 유지하고 std::domain_error를 던진다.
    Angle& operator/=(double scalar);

private:
    explicit Angle(double degrees);
};

Angle abs(Angle angle) noexcept;
Angle wrap180(Angle angle) noexcept;
Angle wrap360(Angle angle) noexcept;
Angle operator*(double scalar, const Angle& angle);

namespace literals
{

Angle operator""_deg(unsigned long long degrees) noexcept;
Angle operator""_deg(long double degrees);
Angle operator""_rad(unsigned long long radians) noexcept;
Angle operator""_rad(long double radians);

}

}
