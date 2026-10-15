#pragma once

#include "math/geometry/spherical_direction.hpp"
#include "math/quantity/length.hpp"

namespace math
{

struct SphericalPoint
{
    // 반지름은 유한하고 0 이상이어야 한다. 조건을 만족하지 않으면 std::invalid_argument를 던진다.
    SphericalPoint(Length radius, SphericalDirection direction);

    const Length radius;
    const SphericalDirection direction;
};

}
