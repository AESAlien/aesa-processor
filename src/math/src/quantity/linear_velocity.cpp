#include "math/quantity/linear_velocity.hpp"

#include <cmath>

namespace math
{
using namespace literals;

const QuantityTolerance<LinearVelocity> LinearVelocity::NONE_TOLERANCE{
    0.0_mps,
    0.0
};

const QuantityTolerance<LinearVelocity> LinearVelocity::DEFAULT_TOLERANCE{
    1e-12_mps,
    1e-9
};

LinearVelocity LinearVelocity::fromMetersPerSecond(double metersPerSecond)
{
    return LinearVelocity(metersPerSecond);
}

double LinearVelocity::mps() const noexcept
{
    return value();
}

LinearVelocity::LinearVelocity(double metersPerSecond)
    : Quantity<LinearVelocity>(metersPerSecond)
{
}

LinearVelocity abs(LinearVelocity velocity) noexcept
{
    return LinearVelocity::fromMetersPerSecond(std::abs(velocity.mps()));
}

namespace literals
{

LinearVelocity operator""_mps(unsigned long long metersPerSecond) noexcept
{
    return LinearVelocity::fromMetersPerSecond(static_cast<double>(metersPerSecond));
}

LinearVelocity operator""_mps(long double metersPerSecond)
{
    return LinearVelocity::fromMetersPerSecond(static_cast<double>(metersPerSecond));
}

}

}
