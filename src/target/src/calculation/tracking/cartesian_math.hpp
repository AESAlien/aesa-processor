#pragma once

#include "domain/target.hpp"

#include <math/geometry/cartesian_point.hpp>
#include <math/geometry/cartesian_velocity.hpp>
#include <math/linear_algebra/vector.hpp>
#include <math/quantity/length.hpp>
#include <math/quantity/linear_velocity.hpp>

#include <cmath>

namespace target
{
namespace cartesian
{

// Convert typed components only at the boundary of dimensionless vector arithmetic.
inline math::Vector toVector(const math::CartesianPoint& position)
{
    return {position.x.m(), position.y.m(), position.z.m()};
}

inline math::Vector toVector(const math::CartesianVelocity& velocity)
{
    return {velocity.x.mps(), velocity.y.mps(), velocity.z.mps()};
}

inline math::CartesianPoint toPosition(const math::Vector& position)
{
    return {math::Length::fromMeters(position[0]),
        math::Length::fromMeters(position[1]),
        math::Length::fromMeters(position[2])};
}

inline math::CartesianVelocity toVelocity(const math::Vector& velocity)
{
    return {math::LinearVelocity::fromMetersPerSecond(velocity[0]),
        math::LinearVelocity::fromMetersPerSecond(velocity[1]),
        math::LinearVelocity::fromMetersPerSecond(velocity[2])};
}

inline math::CartesianPoint position(const Target& target)
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
