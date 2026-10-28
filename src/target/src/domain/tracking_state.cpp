#include "tracking_state.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace target
{
using namespace math::literals;

TargetPrediction::TargetPrediction(
    std::chrono::milliseconds timestamp,
    math::SphericalPosition sphericalPosition,
    std::optional<math::SquareMatrix> positionCovariance)
    : _timestamp(timestamp)
    , _sphericalPosition(std::move(sphericalPosition))
    , _positionCovariance(std::move(positionCovariance))
{
    validate();
}

std::chrono::milliseconds TargetPrediction::timestamp() const noexcept
{
    return _timestamp;
}

const math::SphericalPosition& TargetPrediction::sphericalPosition() const noexcept
{
    return _sphericalPosition;
}

const std::optional<math::SquareMatrix>& TargetPrediction::positionCovariance() const noexcept
{
    return _positionCovariance;
}

void TargetPrediction::validate() const
{
    if (_timestamp.count() < 0
        || !_sphericalPosition.radius.greaterThan(
            0.0_m,
            math::Length::NONE_TOLERANCE))
    {
        throw std::invalid_argument("Prediction requires a positive range and nonnegative time");
    }

    if (_positionCovariance)
    {
        if (_positionCovariance->size() != POSITION_DIMENSION
            || !_positionCovariance->isSymmetric(math::NONE_TOLERANCE))
        {
            throw std::invalid_argument("Prediction covariance requires a finite symmetric 3x3 matrix");
        }
        for (std::size_t index = 0; index < POSITION_DIMENSION; ++index)
        {
            if ((*_positionCovariance)(index, index) < 0.0)
            {
                throw std::invalid_argument("Prediction covariance variances must be nonnegative");
            }
        }
    }
}

TargetTrackingState::TargetTrackingState(
    math::SphericalPosition sphericalPosition,
    math::LinearVelocity dopplerVelocity,
    float power_db,
    std::chrono::milliseconds timestamp,
    math::CartesianPosition position,
    math::CartesianVelocity velocity,
    math::CartesianAcceleration acceleration,
    std::optional<math::SquareMatrix> errorCovariance)
    : _sphericalPosition(std::move(sphericalPosition))
    , _dopplerVelocity(dopplerVelocity)
    , _power_db(power_db)
    , _timestamp(timestamp)
    , _position(std::move(position))
    , _velocity(std::move(velocity))
    , _acceleration(std::move(acceleration))
    , _errorCovariance(std::move(errorCovariance))
{
    validate();
}

const math::SphericalPosition& TargetTrackingState::sphericalPosition() const noexcept
{
    return _sphericalPosition;
}

const math::LinearVelocity& TargetTrackingState::dopplerVelocity() const noexcept
{
    return _dopplerVelocity;
}

float TargetTrackingState::power_db() const noexcept
{
    return _power_db;
}

std::chrono::milliseconds TargetTrackingState::timestamp() const noexcept
{
    return _timestamp;
}

const math::CartesianPosition& TargetTrackingState::position() const noexcept
{
    return _position;
}

const math::CartesianVelocity& TargetTrackingState::velocity() const noexcept
{
    return _velocity;
}

const math::CartesianAcceleration& TargetTrackingState::acceleration() const noexcept
{
    return _acceleration;
}

const std::optional<math::SquareMatrix>& TargetTrackingState::errorCovariance() const noexcept
{
    return _errorCovariance;
}

void TargetTrackingState::validate() const
{
    if (_timestamp.count() < 0
        || !_sphericalPosition.radius.greaterThan(
            0.0_m,
            math::Length::NONE_TOLERANCE)
        || !std::isfinite(_power_db))
    {
        throw std::invalid_argument("Tracking state requires valid observation values and time");
    }

    // Finite components may still produce an unrepresentable total speed.
    static_cast<void>(_velocity.speed());

    if (_errorCovariance)
    {
        if (_errorCovariance->size() != STATE_DIMENSION
            || !_errorCovariance->isSymmetric(math::NONE_TOLERANCE))
        {
            throw std::invalid_argument("Tracking covariance requires a finite symmetric 9x9 matrix");
        }
        for (std::size_t index = 0; index < STATE_DIMENSION; ++index)
        {
            if ((*_errorCovariance)(index, index) < 0.0)
            {
                throw std::invalid_argument("Tracking covariance variances must be nonnegative");
            }
        }
    }
}

} // namespace target
