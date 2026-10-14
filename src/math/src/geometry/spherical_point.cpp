#include "math/geometry/spherical_point.hpp"

#include <stdexcept>

namespace math
{
using namespace literals;

SphericalPoint::SphericalPoint(Distance radius, SphericalDirection direction)
    : radius(radius),
      direction(direction)
{
    if (!radius.isFinite() || radius < 0.0_m)
    {
        throw std::invalid_argument("Spherical point radius must be finite and non-negative");
    }
}

}
