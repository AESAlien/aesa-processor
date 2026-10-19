#include "math/geometry/cartesian_position.hpp"

#include "math/geometry/cartesian_displacement.hpp"
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
    try
    {
        return SphericalPosition(distance(), direction);
    }
    catch (const std::invalid_argument&)
    {
        throw std::overflow_error("Cartesian position radius exceeds the finite Length range");
    }
}

CartesianDisplacement operator-(const CartesianPosition& left, const CartesianPosition& right)
{
    return CartesianDisplacement(
        Length::fromMeters(left.x.m() - right.x.m()),
        Length::fromMeters(left.y.m() - right.y.m()),
        Length::fromMeters(left.z.m() - right.z.m()));
}

}
