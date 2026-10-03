#pragma once

#include "math/distance.hpp"

#include <chrono>
#include <stdexcept>

namespace math
{

// 속도를 초당 미터 단위로 저장하며 음수로 반대 방향을 표현한다.
class Velocity
{
public:
    static Velocity fromMetersPerSecond(double metersPerSecond) noexcept;

    double mps() const noexcept;

    // 초당 미터 단위 절대 차이가 허용오차 이하인지 비교한다.
    // 음수·무한대·NaN 허용오차는 false를 반환한다.
    bool equals(const Velocity& other, const Velocity& absoluteTolerance) const noexcept;

    // 큰 쪽 값의 절댓값에 대한 차이의 비율로 비교한다. 예: 0.01은 1%이다.
    // 음수·무한대·NaN 허용오차와 무한대·NaN 값은 false를 반환한다.
    bool equals(const Velocity& other, double relativeTolerance) const noexcept;

    bool operator<(const Velocity& other) const noexcept;
    bool operator<=(const Velocity& other) const noexcept;
    bool operator>(const Velocity& other) const noexcept;
    bool operator>=(const Velocity& other) const noexcept;

    Velocity operator-() const noexcept;

private:
    explicit Velocity(double metersPerSecond) noexcept;

    double _metersPerSecond = 0.0;
};

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
