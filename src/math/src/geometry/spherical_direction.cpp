#include "math/geometry/spherical_direction.hpp"

#include "math/trigonometry.hpp"
#include "math/linear_algebra/vector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace math
{
SphericalDirection::SphericalDirection(Azimuth azimuth, Elevation elevation) noexcept
    : azimuth(azimuth),
      elevation(elevation)
{
}

Vector SphericalDirection::toCartesianUnitVector() const
{
    const double cosElevation = cos(elevation);
    return Vector{
        cosElevation * sin(azimuth),
        cosElevation * cos(azimuth),
        sin(elevation)
    };
}

SphericalDirection SphericalDirection::fromCartesianUnitVector(const Vector& vector)
{
    if (vector.size() != 3 || !vector.isFinite())
    {
        throw std::invalid_argument("Cartesian direction requires a finite 3D vector");
    }

    const double maxComponent = std::max({
        std::abs(vector(0)),
        std::abs(vector(1)),
        std::abs(vector(2))
    });
    if (maxComponent == 0.0)
    {
        throw std::invalid_argument("Cartesian direction cannot be created from a zero vector");
    }

    const double x = vector(0) / maxComponent;
    const double y = vector(1) / maxComponent;
    const double z = vector(2) / maxComponent;
    const double horizontalNorm = std::hypot(x, y);

    return SphericalDirection(
        Azimuth::fromRadians(std::atan2(x, y)),
        Elevation::fromRadians(std::atan2(z, horizontalNorm)));
}

}
