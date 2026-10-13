#pragma once

#include <math/angle.hpp>
#include <math/distance.hpp>
#include <math/velocity.hpp>

namespace target
{

class Target;
struct TargetTrackingState;
struct TargetPrediction;

namespace validation
{

bool hasValidGeometry(
    const math::Distance& range,
    const math::Angle& azimuth,
    const math::Angle& elevation);
bool hasValidMeasurement(
    const math::Distance& range,
    const math::Angle& azimuth,
    const math::Angle& elevation,
    const math::Velocity& dopplerVelocity);
bool isValidTarget(const Target& target);
bool isValidTrackingState(const TargetTrackingState& state);
bool isValidPrediction(const TargetPrediction& prediction);

} // namespace validation
} // namespace target
