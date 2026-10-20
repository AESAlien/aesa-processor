#pragma once

#include "math/quantity/linear_velocity.hpp"

#include <chrono>

namespace math
{

struct CartesianDisplacement;

struct CartesianVelocity
{
    CartesianVelocity(LinearVelocity x, LinearVelocity y, LinearVelocity z) noexcept;

    LinearVelocity horizontalSpeed() const;
    LinearVelocity speed() const;
    CartesianDisplacement operator*(std::chrono::duration<double> elapsed) const;

    const LinearVelocity x;
    const LinearVelocity y;
    const LinearVelocity z;
};

}
