#include "math/geometry/cartesian_velocity.hpp"

namespace math
{

CartesianVelocity::CartesianVelocity(LinearVelocity x, LinearVelocity y, LinearVelocity z) noexcept
    : x(x),
      y(y),
      z(z)
{
}

}
