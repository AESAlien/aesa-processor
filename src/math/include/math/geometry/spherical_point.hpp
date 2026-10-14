#pragma once

#include "math/geometry/spherical_direction.hpp"
#include "math/quantity/distance.hpp"

namespace math
{

class SphericalPoint
{
public:
    // 반지름은 유한하고 0 이상이어야 한다. 조건을 만족하지 않으면 std::invalid_argument를 던진다.
    SphericalPoint(Distance radius, SphericalDirection direction);

    Distance radius() const noexcept;
    SphericalDirection direction() const noexcept;

private:
    Distance _radius;
    SphericalDirection _direction;
};

}
