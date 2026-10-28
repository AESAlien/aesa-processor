#include "math/geometry/spherical_position.hpp"

#include "math/geometry/cartesian_position.hpp"
#include "math/linear_algebra/vector.hpp"

#include <stdexcept>

namespace math
{
SphericalPosition::SphericalPosition(Length radius, SphericalDirection direction)
    : _radius(radius),
      _direction(direction)
{
    validate();
}

const Length& SphericalPosition::radius() const noexcept
{
    return _radius;
}

const SphericalDirection& SphericalPosition::direction() const noexcept
{
    return _direction;
}

void SphericalPosition::validate() const
{
    if (!_radius.isNonnegative())
    {
        throw std::invalid_argument("Spherical position radius must be non-negative");
    }
}

CartesianPosition SphericalPosition::toCartesian() const
{
    const Vector position = _direction.toCartesianUnitVector() * _radius.m();
    return CartesianPosition(
        Length::fromMeters(position[0]),
        Length::fromMeters(position[1]),
        Length::fromMeters(position[2]));
}

bool SphericalPosition::equals(
    const SphericalPosition& other,
    const QuantityTolerance<Length>& rangeTolerance,
    const QuantityTolerance<Angle>& angleTolerance) const noexcept
{
    return _radius.equals(other._radius, rangeTolerance)
        && _direction.azimuth.isEquivalentTo(other._direction.azimuth, angleTolerance)
        && _direction.elevation.equals(other._direction.elevation, angleTolerance);
}

bool SphericalPosition::operator==(const SphericalPosition& other) const noexcept
{
    return equals(other);
}

bool SphericalPosition::operator!=(const SphericalPosition& other) const noexcept
{
    return !equals(other);
}

}
