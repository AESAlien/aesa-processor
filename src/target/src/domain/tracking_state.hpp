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
    float power_db;
    std::chrono::milliseconds timestamp;
    // Antenna-local Cartesian axes: x at azimuth 0, y at azimuth +90, z upward.
    std::array<math::Distance, 3> position;
    std::array<math::Velocity, 3> velocity;
};

} // namespace target
