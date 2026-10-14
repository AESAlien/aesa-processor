#pragma once

#include "domain/target.hpp"

#include <math/linear_algebra/vector.hpp>
#include <math/quantity/distance.hpp>
#include <math/quantity/velocity.hpp>
#include <math/trigonometry.hpp>

#include <array>

namespace target
{
namespace cartesian
{

// Convert typed components only at the boundary of numerical vector arithmetic.
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

inline std::array<math::Distance, 3> position(
    const math::Distance& slantRange,
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

inline std::array<math::Distance, 3> position(const Target& target)
{
    return position(target.slantRange(), target.azimuth(), target.elevation());
}

} // namespace cartesian
} // namespace target
