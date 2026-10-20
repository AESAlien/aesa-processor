#pragma once

#include "math/quantity/length.hpp"

#include <chrono>

namespace math
{

class Vector;
struct CartesianVelocity;

struct CartesianDisplacement
{
    CartesianDisplacement(Length x, Length y, Length z) noexcept;

    Length distance() const;
    CartesianDisplacement operator+(const CartesianDisplacement& other) const;
    // 각 성분을 미터 단위로 표현한 3차원 벡터를 반환한다.
    Vector toVector() const;
    // 경과 시간이 유한한 양수가 아니거나 계산한 속도가 유한하지 않으면 std::invalid_argument를 던진다.
    CartesianVelocity toVelocity(std::chrono::duration<double> elapsed) const;

    const Length x;
    const Length y;
    const Length z;
};

}
