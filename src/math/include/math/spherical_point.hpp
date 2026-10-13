#pragma once

#include "math/spherical_direction.hpp"
#include "math/distance.hpp"

namespace math
{

class SphericalPoint
{
public:
    // Radius must be finite and non-negative.
    SphericalPoint(Distance radius, SphericalDirection direction);

    Distance radius() const noexcept;
    SphericalDirection direction() const noexcept;

private:
    Distance _radius;
    SphericalDirection _direction;
};

}
