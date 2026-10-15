#include "math/geometry/cartesian_position.hpp"

#include <cmath>

namespace math
{

CartesianPosition::CartesianPosition(Length x, Length y, Length z) noexcept
    : x(x),
      y(y),
      z(z)
{
}

Length CartesianPosition::horizontalDistance() const
{
    return Length::fromMeters(std::hypot(x.m(), y.m()));
}

Length CartesianPosition::distance() const
{
    return Length::fromMeters(std::hypot(x.m(), y.m(), z.m()));
}

}
