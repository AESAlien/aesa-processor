#pragma once

#include "math/quantity/length.hpp"

namespace math
{

struct CartesianPoint
{
    CartesianPoint(Length x, Length y, Length z) noexcept;

    Length horizontalDistance() const;
    Length distance() const;

    const Length x;
    const Length y;
    const Length z;
};

}
