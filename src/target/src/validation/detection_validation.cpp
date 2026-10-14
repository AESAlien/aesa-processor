#include "detection_validation.hpp"

#include "calculation/tracking/cartesian_math.hpp"
#include "domain/target.hpp"
#include "domain/tracking_state.hpp"

#include <cmath>

namespace target
{
namespace validation
{

bool hasValidGeometry(
    const math::Distance& range,
    const math::Angle& azimuth,
    const math::Angle& elevation)
{
    using namespace math::literals;
    return range.isFinite() && range.greaterThan(0.0_m, math::Distance::NONE_TOLERANCE)
        && azimuth.isFinite() && elevation.isFinite()
        && elevation.greaterThanOrEqual(-90.0_deg, math::Angle::NONE_TOLERANCE)
        && elevation.lessThanOrEqual(90.0_deg, math::Angle::NONE_TOLERANCE);
}

bool hasValidMeasurement(
    const math::Distance& range,
    const math::Angle& azimuth,
    const math::Angle& elevation,
    const math::Velocity& dopplerVelocity)
{
    return hasValidGeometry(range, azimuth, elevation)
        && dopplerVelocity.isFinite();
}

bool isValidTarget(const Target& target)
{
    return target.lastObservationTime().count() >= 0
        && hasValidMeasurement(target.slantRange(), target.azimuth(), target.elevation(),
            target.dopplerVelocity());
}

bool isValidTrackingState(const TargetTrackingState& state)
{
    if (state.timestamp.count() < 0
        || !hasValidMeasurement(state.slantRange, state.azimuth, state.elevation,
            state.dopplerVelocity))
    {
        return false;
    }

    const auto position = cartesian::toVector(state.position);
    const auto velocity = cartesian::toVector(state.velocity);
    return position.isFinite() && velocity.isFinite() && std::isfinite(velocity.norm());
}

bool isValidPrediction(const TargetPrediction& prediction)
{
    return prediction.timestamp.count() >= 0
        && hasValidGeometry(prediction.slantRange, prediction.azimuth, prediction.elevation);
}

} // namespace validation
} // namespace target
