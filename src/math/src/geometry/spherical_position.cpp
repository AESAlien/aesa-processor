#include "math/geometry/spherical_position.hpp"

#include "math/geometry/cartesian_position.hpp"
#include "math/linear_algebra/vector.hpp"

#include <stdexcept>

namespace math
{
SphericalPosition::SphericalPosition(Length radius, SphericalDirection direction)
    : radius(radius),
      direction(direction)
{
    if (!radius.isNonnegative())
    {
        throw std::invalid_argument("Spherical position radius must be non-negative");
    }
}

CartesianPosition SphericalPosition::toCartesian() const
{
    const Vector position = direction.toCartesianUnitVector() * radius.m();
    return CartesianPosition(
        Length::fromMeters(position[0]),
        Length::fromMeters(position[1]),
        Length::fromMeters(position[2]));
}

}
