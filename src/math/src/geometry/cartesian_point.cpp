#include "math/geometry/cartesian_point.hpp"

#include <cmath>

namespace math
{

CartesianPoint::CartesianPoint(Length x, Length y, Length z) noexcept
    : x(x),
      y(y),
      z(z)
{
}

Length CartesianPoint::horizontalDistance() const
{
    return Length::fromMeters(std::hypot(x.m(), y.m()));
}

Length CartesianPoint::distance() const
{
    return Length::fromMeters(std::hypot(x.m(), y.m(), z.m()));
}

}
