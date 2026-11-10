#include "math/geometry/cartesian_velocity.hpp"

#include "math/geometry/cartesian_displacement.hpp"

#include <cmath>

namespace math
{

CartesianVelocity::CartesianVelocity(LinearVelocity x, LinearVelocity y, LinearVelocity z) noexcept
    : x(x),
      y(y),
      z(z)
{
}

CartesianDisplacement CartesianVelocity::operator*(std::chrono::duration<double> elapsed) const
{
    return CartesianDisplacement(
        x * elapsed,
        y * elapsed,
        z * elapsed);
}

Speed CartesianVelocity::horizontalSpeed() const
{
    return Speed::fromMetersPerSecond(std::hypot(x.mps(), y.mps()));
}

Speed CartesianVelocity::speed() const
{
    return Speed::fromMetersPerSecond(std::hypot(x.mps(), y.mps(), z.mps()));
}

}
