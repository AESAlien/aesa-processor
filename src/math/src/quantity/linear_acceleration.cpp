#include "math/quantity/linear_acceleration.hpp"

#include <cmath>

namespace math
{
using namespace literals;

const QuantityTolerance<LinearAcceleration> LinearAcceleration::NONE_TOLERANCE{
    0.0_mps2,
    0.0
};

const QuantityTolerance<LinearAcceleration> LinearAcceleration::DEFAULT_TOLERANCE{
    1e-12_mps2,
    1e-9
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
