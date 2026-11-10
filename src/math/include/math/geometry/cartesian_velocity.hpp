#pragma once

#include "math/bounded/speed.hpp"
#include "math/quantity/linear_velocity.hpp"

#include <chrono>

namespace math
{

struct CartesianDisplacement;

struct CartesianVelocity
{
    CartesianVelocity(LinearVelocity x, LinearVelocity y, LinearVelocity z) noexcept;

    Speed horizontalSpeed() const;
    Speed speed() const;
    CartesianDisplacement operator*(std::chrono::duration<double> elapsed) const;

    LinearVelocity x;
    LinearVelocity y;
    LinearVelocity z;
};

}
