#include "detection_validation.hpp"

#include "domain/target.hpp"
#include "domain/tracking_state.hpp"

#include <stdexcept>

namespace target
{
namespace validation
{
namespace
{

bool hasRepresentableSpeed(const math::CartesianVelocity& velocity)
{
    try
    {
        static_cast<void>(velocity.speed());
        return true;
    }
    catch (const std::invalid_argument&)
    {
        return false;
    }
}

} // namespace

bool hasValidGeometry(
    const math::Length& range,
    const math::Angle& elevation)
{
    using namespace math::literals;
    return range.greaterThan(0.0_m, math::Length::NONE_TOLERANCE)
        && elevation.greaterThanOrEqual(-90.0_deg, math::Angle::NONE_TOLERANCE)
        && elevation.lessThanOrEqual(90.0_deg, math::Angle::NONE_TOLERANCE);
}

bool isValidTarget(const Target& target)
{
    return target.lastObservationTime().count() >= 0
        && hasValidGeometry(target.slantRange(), target.elevation());
}

bool isValidTrackingState(const TargetTrackingState& state)
{
    return state.timestamp.count() >= 0
        && hasValidGeometry(state.slantRange, state.elevation)
        && hasRepresentableSpeed(state.velocity);
}

bool isValidPrediction(const TargetPrediction& prediction)
{
    return prediction.timestamp.count() >= 0
        && hasValidGeometry(prediction.slantRange, prediction.elevation);
}

} // namespace validation
} // namespace target
