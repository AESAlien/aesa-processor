#include "math/geometry/cartesian_position.hpp"

#include "math/geometry/spherical_position.hpp"
#include "math/linear_algebra/vector.hpp"

#include <cmath>
#include <stdexcept>

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

SphericalPosition CartesianPosition::toSpherical() const
{
    const Vector position{x.m(), y.m(), z.m()};
    const SphericalDirection direction = SphericalDirection::fromCartesianUnitVector(position);
    const double radiusMeters = position.norm();
    if (!std::isfinite(radiusMeters))
    {
        throw std::overflow_error("Cartesian position radius exceeds the finite Length range");
    }

    return SphericalPosition(Length::fromMeters(radiusMeters), direction);
}

}
