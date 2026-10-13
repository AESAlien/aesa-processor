#include "math/geometry/spherical_direction.hpp"

#include "math/trigonometry.hpp"
#include "math/linear_algebra/vector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace math
{
using namespace literals;

SphericalDirection::SphericalDirection(Angle azimuth, Angle elevation)
    : _azimuth(azimuth),
      _elevation(elevation)
{
    if (!azimuth.isFinite() || azimuth < -180_deg || azimuth > 180_deg)
    {
        throw std::invalid_argument("SphericalDirection azimuth must be in [-180, 180] degrees");
    }
    if (!elevation.isFinite() || elevation < -90_deg || elevation > 90_deg)
    {
        throw std::invalid_argument("SphericalDirection elevation must be in [-90, 90] degrees");
    }
}

Angle SphericalDirection::azimuth() const noexcept
{
    return _azimuth;
}

Angle SphericalDirection::elevation() const noexcept
{
    return _elevation;
}

Vector SphericalDirection::toEnuVector() const
{
    const double cosElevation = cos(_elevation);
    return Vector{
        cosElevation * sin(_azimuth),
        cosElevation * cos(_azimuth),
        sin(_elevation)
    };
}

SphericalDirection SphericalDirection::fromEnuVector(const Vector& vector)
{
    if (vector.size() != 3 || !std::isfinite(vector(0)) ||
        !std::isfinite(vector(1)) || !std::isfinite(vector(2)))
    {
        throw std::invalid_argument("ENU direction requires a finite 3D vector");
    }

    const double maxComponent = std::max({
        std::abs(vector(0)),
        std::abs(vector(1)),
        std::abs(vector(2))
    });
    if (maxComponent == 0.0)
    {
        throw std::invalid_argument("ENU direction cannot be created from a zero vector");
    }

    const double east = vector(0) / maxComponent;
    const double north = vector(1) / maxComponent;
    const double up = vector(2) / maxComponent;
    const double horizontalNorm = std::hypot(east, north);

    const Angle azimuth = Angle::fromRadians(std::atan2(east, north)).wrap180();
    const Angle elevation = Angle::fromRadians(std::atan2(up, horizontalNorm));
    return SphericalDirection(azimuth, elevation);
}

}
