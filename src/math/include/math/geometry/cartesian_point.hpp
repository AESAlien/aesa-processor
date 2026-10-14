#pragma once

#include "math/quantity/distance.hpp"

namespace math
{

struct CartesianPoint
{
    CartesianPoint(Distance x, Distance y, Distance z) noexcept;

    const Distance x;
    const Distance y;
    const Distance z;
};

}
