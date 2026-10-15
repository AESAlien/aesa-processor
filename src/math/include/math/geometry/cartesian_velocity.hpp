#pragma once

#include "math/quantity/linear_velocity.hpp"

namespace math
{

struct CartesianVelocity
{
    CartesianVelocity(LinearVelocity x, LinearVelocity y, LinearVelocity z) noexcept;

    LinearVelocity horizontalSpeed() const;
    LinearVelocity speed() const;

    const LinearVelocity x;
    const LinearVelocity y;
    const LinearVelocity z;
};

}
