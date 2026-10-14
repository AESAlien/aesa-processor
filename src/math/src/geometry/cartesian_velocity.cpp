#include "math/geometry/cartesian_velocity.hpp"

namespace math
{

CartesianVelocity::CartesianVelocity(LinearVelocity x, LinearVelocity y, LinearVelocity z) noexcept
    : _x(x),
      _y(y),
      _z(z)
{
}

LinearVelocity CartesianVelocity::x() const noexcept
{
    return _x;
}

LinearVelocity CartesianVelocity::y() const noexcept
{
    return _y;
}

LinearVelocity CartesianVelocity::z() const noexcept
{
    return _z;
}

}
