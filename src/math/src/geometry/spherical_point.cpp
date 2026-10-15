#include "math/geometry/spherical_point.hpp"

#include <stdexcept>

namespace math
{
SphericalPoint::SphericalPoint(Length radius, SphericalDirection direction)
    : radius(radius),
      direction(direction)
{
    if (!radius.isNonnegative())
    {
        throw std::invalid_argument("Spherical point radius must be non-negative");
    }
}

}
