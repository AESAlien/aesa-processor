#pragma once

#include "math/angle.hpp"
#include "math/distance.hpp"

namespace math
{

class SphericalPoint
{
public:
    // Radius must be finite and non-negative; azimuth must be in [-180, 180]
    // degrees and elevation in [-90, 90] degrees.
    SphericalPoint(Distance radius, Angle azimuth, Angle elevation);

    Distance radius() const noexcept;
    Angle azimuth() const noexcept;
    Angle elevation() const noexcept;

private:
    Distance _radius;
    Angle _azimuth;
    Angle _elevation;
};

}
