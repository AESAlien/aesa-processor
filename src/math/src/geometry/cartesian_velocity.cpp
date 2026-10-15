#include "math/geometry/cartesian_velocity.hpp"

#include <cmath>

namespace math
{

CartesianVelocity::CartesianVelocity(LinearVelocity x, LinearVelocity y, LinearVelocity z) noexcept
    : x(x),
      y(y),
      z(z)
{
}

LinearVelocity CartesianVelocity::horizontalSpeed() const
{
    return LinearVelocity::fromMetersPerSecond(std::hypot(x.mps(), y.mps()));
}

LinearVelocity CartesianVelocity::speed() const
{
    return LinearVelocity::fromMetersPerSecond(std::hypot(x.mps(), y.mps(), z.mps()));
}

}
