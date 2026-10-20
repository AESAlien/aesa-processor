#include "math/quantity/linear_acceleration.hpp"

#include <cmath>

namespace aesa_processor::math
{

const QuantityTolerance<LinearAcceleration> LinearAcceleration::NONE_TOLERANCE{
    LinearAcceleration::fromMetersPerSecondSquared(0.0),
    0.0
};

const QuantityTolerance<LinearAcceleration> LinearAcceleration::DEFAULT_TOLERANCE{
    LinearAcceleration::fromMetersPerSecondSquared(DEFAULT_ABSOLUTE_TOLERANCE),
    DEFAULT_RELATIVE_TOLERANCE
};

LinearAcceleration LinearAcceleration::fromMetersPerSecondSquared(double metersPerSecondSquared)
{
    return LinearAcceleration(metersPerSecondSquared);
}

double LinearAcceleration::mps2() const noexcept
{
    return value();
}

LinearAcceleration::LinearAcceleration(double metersPerSecondSquared)
    : Quantity<LinearAcceleration>(metersPerSecondSquared)
{
}

LinearAcceleration abs(LinearAcceleration acceleration) noexcept
{
    return LinearAcceleration::fromMetersPerSecondSquared(std::abs(acceleration.mps2()));
}

namespace literals
{

LinearAcceleration operator""_mps2(unsigned long long metersPerSecondSquared) noexcept
{
    return LinearAcceleration::fromMetersPerSecondSquared(
        static_cast<double>(metersPerSecondSquared));
}

LinearAcceleration operator""_mps2(long double metersPerSecondSquared)
{
    return LinearAcceleration::fromMetersPerSecondSquared(
        static_cast<double>(metersPerSecondSquared));
}

}

}
