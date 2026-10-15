#include "math/geometry/spherical_position.hpp"

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

}
