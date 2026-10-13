#include "math/geometry/spherical_direction.hpp"

#include "math/trigonometry.hpp"
#include "math/linear_algebra/vector.hpp"

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

    const double horizontalNorm = std::hypot(vector(0), vector(1));
    if (horizontalNorm == 0.0 && vector(2) == 0.0)
    {
        throw std::invalid_argument("ENU direction cannot be created from a zero vector");
    }

    const Angle azimuth = Angle::fromRadians(std::atan2(vector(0), vector(1))).wrap180();
    const Angle elevation = Angle::fromRadians(std::atan2(vector(2), horizontalNorm));
    return SphericalDirection(azimuth, elevation);
}

}
