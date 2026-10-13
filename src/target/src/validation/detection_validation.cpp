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
    return std::isfinite(range.m()) && range > 0.0_m
        && std::isfinite(azimuth.deg()) && std::isfinite(elevation.deg())
        && elevation >= -90.0_deg && elevation <= 90.0_deg;
}

bool hasValidMeasurement(
    const math::Distance& range,
    const math::Angle& azimuth,
    const math::Angle& elevation,
    const math::Velocity& dopplerVelocity)
{
    return hasValidGeometry(range, azimuth, elevation)
        && std::isfinite(dopplerVelocity.mps());
}

bool isValidTarget(const Target& target)
{
    return target.lastObservationTime().count() >= 0
        && hasValidMeasurement(target.slantRange(), target.azimuth(), target.elevation(),
            target.dopplerVelocity());
}

bool isValidTrackingState(const TargetTrackingState& state)
{
    return state.timestamp.count() >= 0
        && hasValidMeasurement(state.slantRange, state.azimuth, state.elevation,
            state.dopplerVelocity)
        && std::isfinite(state.position[0].m())
        && std::isfinite(state.position[1].m())
        && std::isfinite(state.position[2].m())
        && std::isfinite(state.velocity[0].mps())
        && std::isfinite(state.velocity[1].mps())
        && std::isfinite(state.velocity[2].mps())
        && std::isfinite(cartesian::toVector(state.velocity).norm());
}

bool isValidPrediction(const TargetPrediction& prediction)
{
    return prediction.timestamp.count() >= 0
        && hasValidGeometry(prediction.slantRange, prediction.azimuth, prediction.elevation);
}

} // namespace validation
} // namespace target
