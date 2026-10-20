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

LinearVelocity CartesianVelocity::horizontalSpeed() const
{
    return LinearVelocity::fromMetersPerSecond(std::hypot(x.mps(), y.mps()));
}

LinearVelocity CartesianVelocity::speed() const
{
    return LinearVelocity::fromMetersPerSecond(std::hypot(x.mps(), y.mps(), z.mps()));
}

}
