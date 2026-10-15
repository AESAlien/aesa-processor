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
    : azimuth(azimuth),
      elevation(elevation)
{
    if (azimuth < -180_deg || azimuth >= 180_deg)
    {
        throw std::invalid_argument("SphericalDirection azimuth must be in [-180, 180) degrees");
    }
    if (elevation < -90_deg || elevation > 90_deg)
    {
        throw std::invalid_argument("SphericalDirection elevation must be in [-90, 90] degrees");
    }
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

    const Angle azimuth = wrap180(Angle::fromRadians(std::atan2(x, y)));
    const Angle elevation = Angle::fromRadians(std::atan2(z, horizontalNorm));
    return SphericalDirection(azimuth, elevation);
}

}
