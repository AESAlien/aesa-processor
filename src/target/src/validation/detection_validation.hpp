#pragma once

#include <math/quantity/length.hpp>
#include <math/quantity/angle.hpp>

namespace target
{

class Target;
struct TargetTrackingState;
struct TargetPrediction;

namespace validation
{

bool hasValidGeometry(
    const math::Length& range,
    const math::Angle& elevation);
bool isValidTarget(const Target& target);
bool isValidTrackingState(const TargetTrackingState& state);
bool isValidPrediction(const TargetPrediction& prediction);

} // namespace validation
} // namespace target
