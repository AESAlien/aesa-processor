#pragma once

#include "math/geometry/cartesian_velocity.hpp"
#include "math/quantity/linear_acceleration.hpp"

#include <chrono>

namespace math
{

struct CartesianAcceleration
{
    CartesianAcceleration(LinearAcceleration x, LinearAcceleration y, LinearAcceleration z) noexcept;

    LinearAcceleration horizontalAcceleration() const;
    LinearAcceleration acceleration() const;

    template <typename Rep, typename Period>
    CartesianVelocity operator*(const std::chrono::duration<Rep, Period>& elapsed) const
    {
        return CartesianVelocity(x * elapsed, y * elapsed, z * elapsed);
    }

    LinearAcceleration x;
    LinearAcceleration y;
    LinearAcceleration z;
};

}
