#pragma once

#include "math/quantity/quantity_tolerance.hpp"
#include "math/quantity/distance.hpp"

#include <chrono>
#include <stdexcept>

namespace math
{

class Velocity : public Quantity<Velocity>
{
public:
    static const QuantityTolerance<Velocity> DEFAULT_TOLERANCE;

    static Velocity fromMetersPerSecond(double metersPerSecond) noexcept;

    double mps() const noexcept;

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
