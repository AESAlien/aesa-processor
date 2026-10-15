#pragma once

#include <math/geometry/cartesian_point.hpp>
#include <math/geometry/cartesian_velocity.hpp>
#include <math/quantity/angle.hpp>
#include <math/quantity/length.hpp>
#include <math/quantity/linear_velocity.hpp>

#include <chrono>

namespace target
{

struct TargetPrediction
{
    std::chrono::milliseconds timestamp;
    math::Length slantRange;
    math::Angle azimuth;
    math::Angle elevation;
};

struct TargetTrackingState
{
    math::Length slantRange;
    math::Angle azimuth;
    math::Angle elevation;
    math::LinearVelocity dopplerVelocity;
    float power_db;
    std::chrono::milliseconds timestamp;
    // Antenna-local Cartesian axes: x at azimuth 0, y at azimuth +90, z upward.
    math::CartesianPoint position;
    math::CartesianVelocity velocity;
};

} // namespace target
