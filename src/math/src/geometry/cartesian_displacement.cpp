#include "math/geometry/cartesian_displacement.hpp"

#include "math/geometry/cartesian_velocity.hpp"
#include "math/linear_algebra/vector.hpp"

#include <cmath>
#include <stdexcept>

namespace math
{

CartesianDisplacement::CartesianDisplacement(Length x, Length y, Length z) noexcept
    : x(x),
      y(y),
      z(z)
{
}

Length CartesianDisplacement::distance() const
{
    return Length::fromMeters(std::hypot(x.m(), y.m(), z.m()));
}

Vector CartesianDisplacement::toVector() const
{
    return Vector{x.m(), y.m(), z.m()};
}

CartesianVelocity CartesianDisplacement::toVelocity(std::chrono::duration<double> elapsed) const
{
    const double seconds = elapsed.count();
    if (!std::isfinite(seconds) || seconds <= 0.0)
    {
        throw std::invalid_argument("Elapsed time must be finite and positive");
    }

    return CartesianVelocity(
        LinearVelocity::fromMetersPerSecond(x.m() / seconds),
        LinearVelocity::fromMetersPerSecond(y.m() / seconds),
        LinearVelocity::fromMetersPerSecond(z.m() / seconds));
}

}
