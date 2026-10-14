#pragma once

#include "math/quantity/linear_velocity.hpp"

namespace math
{

struct CartesianVelocity
{
    CartesianVelocity(LinearVelocity x, LinearVelocity y, LinearVelocity z) noexcept;

    const LinearVelocity x;
    const LinearVelocity y;
    const LinearVelocity z;
};

}
