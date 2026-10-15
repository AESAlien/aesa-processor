#pragma once

#include "math/quantity/quantity_tolerance.hpp"
#include "math/quantity/length.hpp"

#include <chrono>
#include <stdexcept>

namespace math
{

// 부호를 갖는 초당 미터 단위 속도량으로, 직교 좌표계의 성분과 도플러 측정에도 사용한다.
class LinearVelocity : public Quantity<LinearVelocity>
{
public:
    static const QuantityTolerance<LinearVelocity> DEFAULT_TOLERANCE;
    // 절대·상대 허용오차 없이 비교한다.
    static const QuantityTolerance<LinearVelocity> NONE_TOLERANCE;

    // 저장할 값이 유한하지 않으면 std::invalid_argument를 던진다.
    static LinearVelocity fromMetersPerSecond(double metersPerSecond);

    double mps() const noexcept;

    LinearVelocity operator-() const noexcept;

private:
    explicit LinearVelocity(double metersPerSecond);
};

LinearVelocity abs(LinearVelocity velocity) noexcept;

// 길이를 chrono 시간으로 나눠 초당 미터 속도를 계산한다.
// 시간이 0이면 std::domain_error를 던진다.
template <typename Rep, typename Period>
LinearVelocity operator/(
    const Length& length,
    const std::chrono::duration<Rep, Period>& duration
) {
    const auto seconds = std::chrono::duration<long double>(duration).count();
    if (seconds == 0.0L)
    {
        throw std::domain_error("Cannot divide a length by zero duration");
    }

    return LinearVelocity::fromMetersPerSecond(
        static_cast<double>(static_cast<long double>(length.m()) / seconds));
}

namespace literals
{

LinearVelocity operator""_mps(unsigned long long metersPerSecond) noexcept;
LinearVelocity operator""_mps(long double metersPerSecond);

}

}
