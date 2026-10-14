#include "math/geometry/cartesian_point.hpp"

namespace math
{

CartesianPoint::CartesianPoint(Distance x, Distance y, Distance z) noexcept
    : _x(x),
      _y(y),
      _z(z)
{
}

Distance CartesianPoint::x() const noexcept
{
    return _x;
}

Distance CartesianPoint::y() const noexcept
{
    return _y;
}

Distance CartesianPoint::z() const noexcept
{
    return _z;
}

}
