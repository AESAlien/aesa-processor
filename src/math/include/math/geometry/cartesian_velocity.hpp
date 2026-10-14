#pragma once

#include "math/quantity/linear_velocity.hpp"

namespace math
{

class CartesianVelocity
{
public:
    CartesianVelocity(LinearVelocity x, LinearVelocity y, LinearVelocity z) noexcept;

    LinearVelocity x() const noexcept;
    LinearVelocity y() const noexcept;
    LinearVelocity z() const noexcept;

private:
    LinearVelocity _x;
    LinearVelocity _y;
    LinearVelocity _z;
};

}
