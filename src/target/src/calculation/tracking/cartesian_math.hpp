#pragma once

#include "domain/target.hpp"

#include <math/geometry/cartesian_position.hpp>
#include <math/geometry/cartesian_velocity.hpp>
#include <math/linear_algebra/vector.hpp>
#include <math/quantity/length.hpp>
#include <math/quantity/linear_velocity.hpp>
#include <math/trigonometry.hpp>

namespace target
{
namespace cartesian
{

// Convert typed components only at the boundary of numerical vector arithmetic.
inline math::Vector toVector(const math::CartesianPosition& position)
{
    return {position.x.m(), position.y.m(), position.z.m()};
}

inline math::Vector toVector(const math::CartesianVelocity& velocity)
{
    return {velocity.x.mps(), velocity.y.mps(), velocity.z.mps()};
}

inline math::CartesianPosition toPosition(const math::Vector& position)
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

inline math::CartesianPosition position(
    const math::Length& slantRange,
    const math::Angle& azimuth,
    const math::Angle& elevation)
{
    // Preserve the existing Cartesian conversion for observations and predictions.
    const auto horizontalRange = math::Vector{slantRange.m()}
        * math::cos(elevation);
    const auto horizontalPosition = math::Vector{
        math::cos(azimuth), math::sin(azimuth)}
        * horizontalRange[0];
    const auto altitude = math::Vector{slantRange.m()}
        * math::sin(elevation);
    return toPosition(math::Vector{horizontalPosition[0], horizontalPosition[1], altitude[0]});
}

inline math::CartesianPosition position(const Target& target)
{
    return position(target.slantRange(), target.azimuth(), target.elevation());
}

} // namespace cartesian
} // namespace target
