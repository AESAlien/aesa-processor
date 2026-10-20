#include "math/geometry/cartesian_acceleration.hpp"

#include <cmath>

namespace aesa_processor::math
{

CartesianAcceleration::CartesianAcceleration(
    LinearAcceleration x,
    LinearAcceleration y,
    LinearAcceleration z) noexcept
    : x(x)
    , y(y)
    , z(z)
{
}

LinearAcceleration CartesianAcceleration::horizontalAcceleration() const
{
    return LinearAcceleration::fromMetersPerSecondSquared(std::hypot(x.mps2(), y.mps2()));
}

LinearAcceleration CartesianAcceleration::acceleration() const
{
    return LinearAcceleration::fromMetersPerSecondSquared(
        std::hypot(x.mps2(), y.mps2(), z.mps2()));
}

}
