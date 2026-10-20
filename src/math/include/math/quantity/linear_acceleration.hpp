#pragma once

#include "math/quantity/linear_velocity.hpp"
#include "math/quantity/quantity_tolerance.hpp"

#include <chrono>

namespace math
{

class LinearAcceleration : public Quantity<LinearAcceleration>
{
public:
    using Quantity<LinearAcceleration>::operator*;

    static const QuantityTolerance<LinearAcceleration> DEFAULT_TOLERANCE;
    static const QuantityTolerance<LinearAcceleration> NONE_TOLERANCE;

    static LinearAcceleration fromMetersPerSecondSquared(double metersPerSecondSquared);

    double mps2() const noexcept;

    template <typename Rep, typename Period>
    LinearVelocity operator*(const std::chrono::duration<Rep, Period>& duration) const
    {
        const auto seconds = std::chrono::duration<long double>(duration).count();
        return LinearVelocity::fromMetersPerSecond(
            static_cast<double>(static_cast<long double>(mps2()) * seconds));
    }

private:
    friend class Quantity<LinearAcceleration>;

    explicit LinearAcceleration(double metersPerSecondSquared);
};

LinearAcceleration abs(LinearAcceleration acceleration) noexcept;

namespace literals
{

LinearAcceleration operator""_mps2(unsigned long long metersPerSecondSquared) noexcept;
LinearAcceleration operator""_mps2(long double metersPerSecondSquared);

}

}
