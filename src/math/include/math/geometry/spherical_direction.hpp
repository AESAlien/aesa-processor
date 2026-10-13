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

    // Converts to a Cartesian unit vector using azimuth 0 along +y.
    Vector toCartesianUnitVector() const;
    // Converts a finite, non-zero 3D Cartesian vector to a direction; its magnitude is ignored.
    static SphericalDirection fromCartesianUnitVector(const Vector& vector);

private:
    Angle _azimuth;
    Angle _elevation;
};

}
