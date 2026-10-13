#include "math/spherical_point.hpp"

#include <stdexcept>

namespace math
{
using namespace literals;

SphericalPoint::SphericalPoint(Distance radius, SphericalDirection direction)
    : _radius(radius),
      _direction(direction)
{
    if (!radius.isFinite() || radius < 0.0_m)
    {
        throw std::invalid_argument("Spherical point radius must be finite and non-negative");
    }
}

Distance SphericalPoint::radius() const noexcept
{
    return _radius;
}

SphericalDirection SphericalPoint::direction() const noexcept
{
    return _direction;
}

}
