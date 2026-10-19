#include "tracking_state.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace target
{

TargetPrediction::TargetPrediction(
    std::chrono::milliseconds timestamp,
    math::SphericalPosition sphericalPosition)
    : timestamp(timestamp)
    , sphericalPosition(std::move(sphericalPosition))
{
    validate();
}

void TargetPrediction::validate() const
{
    using namespace math::literals;
    if (timestamp.count() < 0
        || !sphericalPosition.radius.greaterThan(0.0_m, math::Length::NONE_TOLERANCE))
    {
        throw std::invalid_argument("Prediction requires a positive range and nonnegative time");
    }
}

TargetTrackingState::TargetTrackingState(
    math::SphericalPosition sphericalPosition,
    math::LinearVelocity dopplerVelocity,
    float power_db,
    std::chrono::milliseconds timestamp,
    math::CartesianPosition position,
    math::CartesianVelocity velocity)
    : sphericalPosition(std::move(sphericalPosition))
    , dopplerVelocity(dopplerVelocity)
    , power_db(power_db)
    , timestamp(timestamp)
    , position(std::move(position))
    , velocity(std::move(velocity))
{
    validate();
}

void TargetTrackingState::validate() const
{
    using namespace math::literals;
    if (timestamp.count() < 0
        || !sphericalPosition.radius.greaterThan(0.0_m, math::Length::NONE_TOLERANCE)
        || !std::isfinite(power_db))
    {
        throw std::invalid_argument("Tracking state requires valid observation values and time");
    }

    // Finite components may still produce an unrepresentable total speed.
    static_cast<void>(velocity.speed());
}

} // namespace target
