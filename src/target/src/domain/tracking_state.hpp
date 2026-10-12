#pragma once

#include <math/angle.hpp>
#include <math/distance.hpp>
#include <math/velocity.hpp>

#include <array>
#include <chrono>

namespace target
{

struct TargetPrediction
{
    std::chrono::milliseconds timestamp;
    math::Distance slantRange;
    math::Angle azimuth;
    math::Angle elevation;
};

struct TargetTrackingState
{
    math::Distance slantRange;
    math::Angle azimuth;
    math::Angle elevation;
    math::Velocity dopplerVelocity;
    float powerDb;
    std::chrono::milliseconds timestamp;
    // Antenna-local Cartesian axes: x at azimuth 0, y at azimuth +90, z upward.
    std::array<double, 3> positionMeters;
    std::array<double, 3> velocityMetersPerSecond;
};

} // namespace target
