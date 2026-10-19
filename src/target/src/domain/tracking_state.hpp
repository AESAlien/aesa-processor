#pragma once

#include <math/geometry/cartesian_position.hpp>
#include <math/geometry/cartesian_velocity.hpp>
#include <math/geometry/spherical_position.hpp>
#include <math/quantity/linear_velocity.hpp>

#include <chrono>

namespace target
{

struct TargetPrediction
{
    TargetPrediction(
        std::chrono::milliseconds timestamp,
        math::SphericalPosition sphericalPosition);

    const std::chrono::milliseconds timestamp;
    const math::SphericalPosition sphericalPosition;

private:
    void validate() const;
};

struct TargetTrackingState
{
    TargetTrackingState(
        math::SphericalPosition sphericalPosition,
        math::LinearVelocity dopplerVelocity,
        float power_db,
        std::chrono::milliseconds timestamp,
        math::CartesianPosition position,
        math::CartesianVelocity velocity);

    const math::SphericalPosition sphericalPosition;
    const math::LinearVelocity dopplerVelocity;
    const float power_db;
    const std::chrono::milliseconds timestamp;
    // Match math geometry: y at azimuth 0, x at azimuth +90, z upward.
    const math::CartesianPosition position;
    const math::CartesianVelocity velocity;

private:
    void validate() const;
};

} // namespace target
