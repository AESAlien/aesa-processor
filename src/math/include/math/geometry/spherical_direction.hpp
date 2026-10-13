#pragma once

#include "math/quantity/angle.hpp"

namespace math
{

class Vector;

class SphericalDirection
{
public:
    // 방위각은 [-180, 180)도, 고각은 [-90, 90]도 범위여야 한다. 무한대·NaN 값이나 범위를 벗어난 값이면 std::invalid_argument를 던진다.
    SphericalDirection(Angle azimuth, Angle elevation);

    Angle azimuth() const noexcept;
    Angle elevation() const noexcept;

    // 방위각 0도의 방향을 +y축으로 하는 직교 좌표계의 단위벡터를 반환한다.
    Vector toCartesianUnitVector() const;
    // 벡터의 크기를 무시하고 방향을 반환한다. 3차원이 아니거나 무한대·NaN 원소를 포함하거나 영벡터이면 std::invalid_argument를 던진다.
    static SphericalDirection fromCartesianUnitVector(const Vector& vector);

private:
    Angle _azimuth;
    Angle _elevation;
};

}
