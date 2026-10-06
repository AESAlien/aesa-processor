#pragma once

namespace math
{

extern const double PI;

class Angle
{
public:
    static Angle fromDegrees(double degrees) noexcept;
    static Angle fromRadians(double radians) noexcept;

    double deg() const noexcept;
    double rad() const noexcept;

    // 원본을 유지하며 [-180, 180)도 범위로 정규화한 새 각도를 반환한다.
    Angle wrap180() const noexcept;

    // 도 단위 절대 차이가 허용오차 이하인지 비교한다.
    // 음수·무한대·NaN 허용오차는 false를 반환한다.
    bool equals(const Angle& other, const Angle& absoluteTolerance) const noexcept;

    // 큰 쪽 값의 절댓값에 대한 차이의 비율로 비교한다. 예: 0.01은 1%이다.
    // 음수·무한대·NaN 허용오차와 무한대·NaN 값은 false를 반환한다.
    bool equals(const Angle& other, double relativeTolerance) const noexcept;

    bool operator<(const Angle& other) const noexcept;
    bool operator<=(const Angle& other) const noexcept;
    bool operator>(const Angle& other) const noexcept;
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
    // 0이면 원본을 유지하고 std::domain_error를 던진다.
    Angle& operator/=(double scalar);

private:
    explicit Angle(double degrees) noexcept;

    double _degrees = 0.0;
};

Angle operator*(double scalar, const Angle& angle) noexcept;

namespace literals
{

Angle operator""_deg(unsigned long long degrees) noexcept;
Angle operator""_deg(long double degrees) noexcept;

}

}
