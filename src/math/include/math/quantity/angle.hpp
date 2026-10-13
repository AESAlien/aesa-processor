#pragma once

#include "math/quantity/quantity_tolerance.hpp"

namespace math
{

extern const double PI;

class Angle : public Quantity
{
public:
    static const QuantityTolerance<Angle> DEFAULT_TOLERANCE;

    static Angle fromDegrees(double degrees) noexcept;
    static Angle fromRadians(double radians) noexcept;

    double deg() const noexcept;
    double rad() const noexcept;

    // 원본을 유지하며 [-180, 180)도 범위로 정규화한 새 각도를 반환한다.
    Angle wrap180() const noexcept;

    // 원본을 유지하며 [-90, 90)도 범위로 정규화한 새 각도를 반환한다.
    Angle wrap90() const noexcept;

    // 지정한 절대오차 또는 상대오차 범위 안이면 true를 반환한다. 무한대·NaN 값은 false를 반환한다.
    // 허용오차를 생략하면 DEFAULT 절대·상대오차를 적용한다.
    bool equals(const Angle& other, const QuantityTolerance<Angle>& tolerance = DEFAULT_TOLERANCE) const noexcept;

    // DEFAULT 절대오차 또는 상대오차 범위 안이면 true를 반환한다. 무한대·NaN 값은 false를 반환한다.
    bool operator==(const Angle& other) const noexcept;
    // ==의 결과를 반전한다. 무한대·NaN 값은 자기 자신과도 다르다.
    bool operator!=(const Angle& other) const noexcept;

    // DEFAULT 오차 범위 밖에서 더 작으면 true를 반환한다.
    bool operator<(const Angle& other) const noexcept;
    // 더 작거나 DEFAULT 오차 범위 안이면 true를 반환한다.
    bool operator<=(const Angle& other) const noexcept;
    // DEFAULT 오차 범위 밖에서 더 크면 true를 반환한다.
    bool operator>(const Angle& other) const noexcept;
    // 더 크거나 DEFAULT 오차 범위 안이면 true를 반환한다.
    bool operator>=(const Angle& other) const noexcept;

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
