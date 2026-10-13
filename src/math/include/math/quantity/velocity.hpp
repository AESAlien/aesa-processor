#pragma once

#include "math/tolerance.hpp"
#include "math/quantity/distance.hpp"

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
    bool isFinite() const noexcept;
    // 저장된 값이 0 이상인지 검사한다. NaN은 false이다.
    bool isNonnegative() const noexcept;

    // 단위를 보존한 절대·상대오차 정책을 적용한다. 비유한 값은 false이다.
    bool equals(const Velocity& other, const Tolerance<Velocity>& tolerance) const noexcept;

    // 공통 DEFAULT 절대·상대오차를 함께 적용한다. 무한대·NaN 값은 false이다.
    bool operator==(const Velocity& other) const noexcept;
    // ==의 부정이다. 무한대·NaN 값은 자기 자신과도 다르다.
    bool operator!=(const Velocity& other) const noexcept;

    // DEFAULT 오차 범위 밖에서 더 작으면 true이다.
    bool operator<(const Velocity& other) const noexcept;
    // 더 작거나 DEFAULT 오차 범위 안이면 true이다.
    bool operator<=(const Velocity& other) const noexcept;
    // DEFAULT 오차 범위 밖에서 더 크면 true이다.
    bool operator>(const Velocity& other) const noexcept;
    // 더 크거나 DEFAULT 오차 범위 안이면 true이다.
    bool operator>=(const Velocity& other) const noexcept;

    Velocity operator-() const noexcept;

private:
    explicit Velocity(double metersPerSecond) noexcept;

    double _metersPerSecond = 0.0;
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
