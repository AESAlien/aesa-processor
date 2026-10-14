#pragma once

#include "math/quantity/quantity_tolerance.hpp"
#include "math/quantity/distance.hpp"

#include <chrono>
#include <stdexcept>

namespace math
{

// Signed meters-per-second quantity, also used for Cartesian components and Doppler measurements.
class LinearVelocity : public Quantity<LinearVelocity>
{
public:
    static const QuantityTolerance<LinearVelocity> DEFAULT_TOLERANCE;
    // 절대·상대 허용오차 없이 비교한다. 무한대·NaN 값은 같다고 판단하지 않는다.
    static const QuantityTolerance<LinearVelocity> NONE_TOLERANCE;

    static LinearVelocity fromMetersPerSecond(double metersPerSecond) noexcept;

    double mps() const noexcept;

    LinearVelocity operator-() const noexcept;

private:
    explicit LinearVelocity(double metersPerSecond) noexcept;
};

LinearVelocity abs(LinearVelocity velocity) noexcept;

// 거리를 chrono 시간으로 나눠 초당 미터 속도를 계산한다.
// 시간이 0이면 std::domain_error를 던진다.
template <typename Rep, typename Period>
LinearVelocity operator/(
    const Distance& distance,
    const std::chrono::duration<Rep, Period>& duration
) {
    const auto seconds = std::chrono::duration<long double>(duration).count();
    if (seconds == 0.0L)
    {
        throw std::domain_error("Cannot divide a distance by zero duration");
    }

    return LinearVelocity::fromMetersPerSecond(
        static_cast<double>(static_cast<long double>(distance.m()) / seconds));
}

namespace literals
{

LinearVelocity operator""_mps(unsigned long long metersPerSecond) noexcept;
LinearVelocity operator""_mps(long double metersPerSecond) noexcept;

}

}
