#pragma once

#include "math/geometry/spherical_direction.hpp"
#include "math/quantity/length.hpp"

namespace math
{

struct CartesianPosition;

struct SphericalPosition
{
    // 반지름은 유한하고 0 이상이어야 한다. 조건을 만족하지 않으면 std::invalid_argument를 던진다.
    SphericalPosition(Length radius, SphericalDirection direction);

    // 방위각 0도는 +y축, +90도는 +x축, 양의 고각은 +z축인 직교 위치를 반환한다.
    // 반지름이 0이면 방향과 관계없이 원점을 반환한다.
    CartesianPosition toCartesian() const;

    const Length radius;
    const SphericalDirection direction;
};

}
