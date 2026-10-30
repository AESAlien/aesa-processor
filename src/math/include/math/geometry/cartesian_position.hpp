#pragma once

#include "math/bounded/distance.hpp"
#include "math/quantity/length.hpp"

namespace math
{

struct SphericalPosition;
struct CartesianDisplacement;

struct CartesianPosition
{
    CartesianPosition(Length x, Length y, Length z) noexcept;

    Distance horizontalDistance() const;
    Distance distance() const;
    CartesianPosition operator+(const CartesianDisplacement& displacement) const;
    CartesianPosition operator-(const CartesianDisplacement& displacement) const;

    // 오른쪽 위치에서 왼쪽 위치로 향하는 변위를 반환한다.
    // 성분이 유한한 Length 범위를 넘으면 std::invalid_argument를 던진다.
    CartesianDisplacement operator-(const CartesianPosition& other) const;

    // 방위각 0도를 +y축으로 하며, 방위각은 [0, 360)도 범위로 반환한다.
    // 원점은 방향을 정의할 수 없어 std::invalid_argument를 던진다.
    // 계산한 거리가 유한한 Distance 범위를 넘으면 std::overflow_error를 던진다.
    SphericalPosition toSpherical() const;

    Length x;
    Length y;
    Length z;
};

}
