#pragma once

#include "math/quantity/angle.hpp"

namespace math
{

class Vector;

class SphericalDirection
{
public:
    // Azimuth must be in [-180, 180] degrees and elevation in [-90, 90] degrees.
    SphericalDirection(Angle azimuth, Angle elevation);

    Angle azimuth() const noexcept;
    Angle elevation() const noexcept;

    // Converts to a unit vector with ENU component order: east, north, up.
    Vector toEnuVector() const;
    // Converts a finite, non-zero 3D ENU vector to a direction.
    static SphericalDirection fromEnuVector(const Vector& vector);

private:
    Angle _azimuth;
    Angle _elevation;
};

}
