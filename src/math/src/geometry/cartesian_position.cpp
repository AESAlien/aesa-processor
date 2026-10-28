#include "math/geometry/cartesian_position.hpp"

#include "math/geometry/cartesian_displacement.hpp"
#include "math/geometry/distance.hpp"
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

Distance CartesianPosition::horizontalDistance() const
{
    return Distance::fromMeters(std::hypot(x.m(), y.m()));
}

Distance CartesianPosition::distance() const
{
    return Distance::fromMeters(std::hypot(x.m(), y.m(), z.m()));
}

SphericalPosition CartesianPosition::toSpherical() const
{
    const Vector position{x.m(), y.m(), z.m()};
    const SphericalDirection direction = SphericalDirection::fromCartesianUnitVector(position);
    try
    {
        return SphericalPosition{distance(), direction};
    }
    catch (const std::invalid_argument&)
    {
        throw std::overflow_error("Cartesian position distance exceeds the finite Distance range");
    }
}

CartesianPosition CartesianPosition::operator+(const CartesianDisplacement& displacement) const
{
    return CartesianPosition(
        x + displacement.x,
        y + displacement.y,
        z + displacement.z);
}

CartesianPosition CartesianPosition::operator-(const CartesianDisplacement& displacement) const
{
    return CartesianPosition(
        x - displacement.x,
        y - displacement.y,
        z - displacement.z);
}

CartesianDisplacement CartesianPosition::operator-(const CartesianPosition& other) const
{
    return CartesianDisplacement(
        x - other.x,
        y - other.y,
        z - other.z);
}

}
