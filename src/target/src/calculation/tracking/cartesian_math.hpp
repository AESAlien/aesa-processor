#pragma once

#include "domain/target.hpp"

#include <math/linear_algebra/vector.hpp>
#include <math/quantity/distance.hpp>
#include <math/quantity/velocity.hpp>

#include <array>
#include <cmath>

namespace target
{
namespace cartesian
{

// Convert typed components only at the boundary of dimensionless vector arithmetic.
inline math::Vector toVector(const std::array<math::Distance, 3>& position)
{
    return {position[0].m(), position[1].m(), position[2].m()};
}

inline math::Vector toVector(const std::array<math::Velocity, 3>& velocity)
{
    return {velocity[0].mps(), velocity[1].mps(), velocity[2].mps()};
}

inline std::array<math::Distance, 3> toPosition(const math::Vector& position)
{
    return {math::Distance::fromMeters(position[0]),
        math::Distance::fromMeters(position[1]),
        math::Distance::fromMeters(position[2])};
}

inline std::array<math::Velocity, 3> toVelocity(const math::Vector& velocity)
{
    return {math::Velocity::fromMetersPerSecond(velocity[0]),
        math::Velocity::fromMetersPerSecond(velocity[1]),
        math::Velocity::fromMetersPerSecond(velocity[2])};
}

inline std::array<math::Distance, 3> position(const Target& target)
{
    // The current math API has no trigonometric functions.
    const auto horizontalRange = math::Vector{target.slantRange().m()}
        * std::cos(target.elevation().rad());
    const auto horizontalPosition = math::Vector{
        std::cos(target.azimuth().rad()), std::sin(target.azimuth().rad())}
        * horizontalRange[0];
    const auto altitude = math::Vector{target.slantRange().m()}
        * std::sin(target.elevation().rad());
    return toPosition(math::Vector{horizontalPosition[0], horizontalPosition[1], altitude[0]});
}

} // namespace cartesian
} // namespace target
