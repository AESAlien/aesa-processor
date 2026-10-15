#pragma once

#include <math/quantity/angle.hpp>
#include <math/quantity/length.hpp>
#include <math/quantity/linear_velocity.hpp>

namespace target
{

class Target;
struct TargetTrackingState;
struct TargetPrediction;

namespace validation
{

bool hasValidGeometry(
    const math::Length& range,
    const math::Angle& azimuth,
    const math::Angle& elevation);
bool hasValidMeasurement(
    const math::Length& range,
    const math::Angle& azimuth,
    const math::Angle& elevation,
    const math::LinearVelocity& dopplerVelocity);
bool isValidTarget(const Target& target);
bool isValidTrackingState(const TargetTrackingState& state);
bool isValidPrediction(const TargetPrediction& prediction);

} // namespace validation
} // namespace target
