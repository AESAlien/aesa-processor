#pragma once

#include "math/geometry/azimuth.hpp"
#include "math/geometry/elevation.hpp"

namespace math
{

class Vector;

struct SphericalDirection
{
    // 방위각은 [0, 360)도로 정규화한다. 고각이 [-90, 90]도 범위를 벗어나면 std::invalid_argument를 던진다.
    SphericalDirection(Azimuth azimuth, Elevation elevation) noexcept;

    // 방위각 0도의 방향을 +y축으로 하는 직교 좌표계의 단위벡터를 반환한다.
    Vector toCartesianUnitVector() const;
    // 벡터의 크기를 무시하고 방향을 반환한다. 3차원이 아니거나 무한대·NaN 원소를 포함하거나 영벡터이면 std::invalid_argument를 던진다.
    static SphericalDirection fromCartesianUnitVector(const Vector& vector);

    Azimuth azimuth;
    Elevation elevation;
};

}
