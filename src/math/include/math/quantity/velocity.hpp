#pragma once

#include "math/quantity/quantity_tolerance.hpp"
#include "math/quantity/distance.hpp"

#include <chrono>
#include <stdexcept>

namespace math
{

class Velocity : public Quantity
{
public:
    static const QuantityTolerance<Velocity> DEFAULT_TOLERANCE;

    static Velocity fromMetersPerSecond(double metersPerSecond) noexcept;

    double mps() const noexcept;

    // 지정한 절대오차 또는 상대오차 범위 안이면 true를 반환한다. 무한대·NaN 값은 false를 반환한다.
    // 허용오차를 생략하면 DEFAULT 절대·상대오차를 적용한다.
    bool equals(const Velocity& other, const QuantityTolerance<Velocity>& tolerance = DEFAULT_TOLERANCE) const noexcept;

    // DEFAULT 절대오차 또는 상대오차 범위 안이면 true를 반환한다. 무한대·NaN 값은 false를 반환한다.
    bool operator==(const Velocity& other) const noexcept;
    // ==의 결과를 반전한다. 무한대·NaN 값은 자기 자신과도 다르다.
    bool operator!=(const Velocity& other) const noexcept;

    // DEFAULT 오차 범위 밖에서 더 작으면 true를 반환한다.
    bool operator<(const Velocity& other) const noexcept;
    // 더 작거나 DEFAULT 오차 범위 안이면 true를 반환한다.
    bool operator<=(const Velocity& other) const noexcept;
    // DEFAULT 오차 범위 밖에서 더 크면 true를 반환한다.
    bool operator>(const Velocity& other) const noexcept;
    // 더 크거나 DEFAULT 오차 범위 안이면 true를 반환한다.
    bool operator>=(const Velocity& other) const noexcept;

    Velocity operator-() const noexcept;

private:
    explicit Velocity(double metersPerSecond) noexcept;
};

Velocity abs(Velocity velocity) noexcept;

// 거리를 chrono 시간으로 나눠 초당 미터 속도를 계산한다.
// 시간이 0이면 std::domain_error를 던진다.
template <typename Rep, typename Period>
Velocity operator/(
    const Distance& distance,
    const std::chrono::duration<Rep, Period>& duration
) {
    const auto seconds = std::chrono::duration<long double>(duration).count();
    if (seconds == 0.0L)
    {
        throw std::domain_error("Cannot divide a distance by zero duration");
    }

    return Velocity::fromMetersPerSecond(
        static_cast<double>(static_cast<long double>(distance.m()) / seconds));
}

namespace literals
{

Velocity operator""_mps(unsigned long long metersPerSecond) noexcept;
Velocity operator""_mps(long double metersPerSecond) noexcept;

}

}
