#include "math/quantity/linear_velocity.hpp"

#include <cmath>

namespace math
{
using namespace literals;

const QuantityTolerance<LinearVelocity> LinearVelocity::NONE_TOLERANCE{
    LinearVelocity::fromMetersPerSecond(0.0),
    0.0
};

const QuantityTolerance<LinearVelocity> LinearVelocity::DEFAULT_TOLERANCE{
    LinearVelocity::fromMetersPerSecond(DEFAULT_ABSOLUTE_TOLERANCE),
    DEFAULT_RELATIVE_TOLERANCE
};

LinearVelocity LinearVelocity::fromMetersPerSecond(double metersPerSecond)
{
    return LinearVelocity(metersPerSecond);
}

double LinearVelocity::mps() const noexcept
{
    return value();
}

LinearVelocity LinearVelocity::operator-() const noexcept
{
    return LinearVelocity(-value());
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
