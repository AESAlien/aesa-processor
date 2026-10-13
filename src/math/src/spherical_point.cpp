#include "math/spherical_point.hpp"

#include <stdexcept>

namespace math
{
using namespace literals;

SphericalPoint::SphericalPoint(Distance radius, Angle azimuth, Angle elevation)
    : _radius(radius),
      _azimuth(azimuth),
      _elevation(elevation)
{
    if (!radius.isFinite() || radius < 0.0_m)
    {
        throw std::invalid_argument("Spherical point radius must be finite and non-negative");
    }
    if (!azimuth.isFinite() || azimuth < -180_deg || azimuth > 180_deg)
    {
        throw std::invalid_argument("Spherical point azimuth must be in [-180, 180] degrees");
    }
    if (!elevation.isFinite() || elevation < -90_deg || elevation > 90_deg)
    {
        throw std::invalid_argument("Spherical point elevation must be in [-90, 90] degrees");
    }
}

Distance SphericalPoint::radius() const noexcept
{
    return _radius;
}

Angle SphericalPoint::azimuth() const noexcept
{
    return _azimuth;
}

Angle SphericalPoint::elevation() const noexcept
{
    return _elevation;
}

}
