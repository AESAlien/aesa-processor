#include "tracking_state.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace aesa_processor::target
{

TargetPrediction::TargetPrediction(
    std::chrono::milliseconds timestamp,
    aesa_processor::math::SphericalPosition sphericalPosition,
    std::optional<aesa_processor::math::SquareMatrix> positionCovariance)
    : timestamp(timestamp)
    , sphericalPosition(std::move(sphericalPosition))
    , positionCovariance(std::move(positionCovariance))
{
    validate();
}

void TargetPrediction::validate() const
{
    using namespace aesa_processor::math::literals;
    if (timestamp.count() < 0
        || !sphericalPosition.radius.greaterThan(0.0_m, aesa_processor::math::Length::NONE_TOLERANCE))
    {
        throw std::invalid_argument("Prediction requires a positive range and nonnegative time");
    }

    if (positionCovariance)
    {
        if (positionCovariance->size() != POSITION_DIMENSION
            || !positionCovariance->isSymmetric(aesa_processor::math::NONE_TOLERANCE))
        {
            throw std::invalid_argument("Prediction covariance requires a finite symmetric 3x3 matrix");
        }
        for (std::size_t index = 0; index < POSITION_DIMENSION; ++index)
        {
            if ((*positionCovariance)(index, index) < 0.0)
            {
                throw std::invalid_argument("Prediction covariance variances must be nonnegative");
            }
        }
    }
}

TargetTrackingState::TargetTrackingState(
    aesa_processor::math::SphericalPosition sphericalPosition,
    aesa_processor::math::LinearVelocity dopplerVelocity,
    float power_db,
    std::chrono::milliseconds timestamp,
    aesa_processor::math::CartesianPosition position,
    aesa_processor::math::CartesianVelocity velocity,
    aesa_processor::math::CartesianAcceleration acceleration,
    std::optional<aesa_processor::math::SquareMatrix> errorCovariance)
    : sphericalPosition(std::move(sphericalPosition))
    , dopplerVelocity(dopplerVelocity)
    , power_db(power_db)
    , timestamp(timestamp)
    , position(std::move(position))
    , velocity(std::move(velocity))
    , acceleration(std::move(acceleration))
    , errorCovariance(std::move(errorCovariance))
{
    validate();
}

void TargetTrackingState::validate() const
{
    using namespace aesa_processor::math::literals;
    if (timestamp.count() < 0
        || !sphericalPosition.radius.greaterThan(0.0_m, aesa_processor::math::Length::NONE_TOLERANCE)
        || !std::isfinite(power_db))
    {
        throw std::invalid_argument("Tracking state requires valid observation values and time");
    }

    // Finite components may still produce an unrepresentable total speed.
    static_cast<void>(velocity.speed());

    if (errorCovariance)
    {
        if (errorCovariance->size() != STATE_DIMENSION
            || !errorCovariance->isSymmetric(aesa_processor::math::NONE_TOLERANCE))
        {
            throw std::invalid_argument("Tracking covariance requires a finite symmetric 9x9 matrix");
        }
        for (std::size_t index = 0; index < STATE_DIMENSION; ++index)
        {
            if ((*errorCovariance)(index, index) < 0.0)
            {
                throw std::invalid_argument("Tracking covariance variances must be nonnegative");
            }
        }
    }
}

} // namespace aesa_processor::target
