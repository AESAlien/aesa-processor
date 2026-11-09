#include "track_state.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace target
{
using namespace math::literals;

TargetPrediction::TargetPrediction(
    std::chrono::milliseconds timestamp,
    math::SphericalPosition sphericalPosition,
    std::optional<math::SquareMatrix> positionCovariance
) : _timestamp(timestamp),
    _sphericalPosition(std::move(sphericalPosition)),
    _positionCovariance(std::move(positionCovariance))
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

void TargetPrediction::validate() const
{
    if (_timestamp.count() < 0 ||
        _sphericalPosition.radius.isNonpositive()
    ) {
        throw std::invalid_argument("Prediction requires a positive range and nonnegative time");
    }

    if (_positionCovariance)
    {
        if (_positionCovariance->size() != POSITION_DIMENSION)
        {
            throw std::invalid_argument("Prediction covariance requires a 3x3 matrix");
        }
        try
        {
            static_cast<void>(math::cholesky(*_positionCovariance));
        }
        catch (const std::domain_error&)
        {
            throw std::invalid_argument("Prediction covariance must be positive definite");
        }
    }
}

TrackState::TrackState(
    math::SphericalPosition sphericalPosition,
    math::LinearVelocity dopplerVelocity,
    float power_db,
    std::chrono::milliseconds timestamp,
    math::CartesianPosition position,
    math::CartesianVelocity velocity,
    math::CartesianAcceleration acceleration,
    std::optional<math::SquareMatrix> errorCovariance
) : _sphericalPosition(std::move(sphericalPosition)),
    _dopplerVelocity(dopplerVelocity),
    _power_db(power_db),
    _timestamp(timestamp),
    _position(std::move(position)),
    _velocity(std::move(velocity)),
    _acceleration(std::move(acceleration)),
    _errorCovariance(std::move(errorCovariance))
{
    validate();
}

const math::SphericalPosition& TrackState::sphericalPosition() const noexcept
{
    return _sphericalPosition;
}

const math::LinearVelocity& TrackState::dopplerVelocity() const noexcept
{
    return _dopplerVelocity;
}

float TrackState::power_db() const noexcept
{
    return _power_db;
}

std::chrono::milliseconds TrackState::timestamp() const noexcept
{
    return _timestamp;
}

const math::CartesianPosition& TrackState::position() const noexcept
{
    return _position;
}

const math::CartesianVelocity& TrackState::velocity() const noexcept
{
    return _velocity;
}

const math::CartesianAcceleration& TrackState::acceleration() const noexcept
{
    return _acceleration;
}

const std::optional<math::SquareMatrix>& TrackState::errorCovariance() const noexcept
{
    return _errorCovariance;
}

void TrackState::validate() const
{
    if (_timestamp.count() < 0 ||
        _sphericalPosition.radius.isNonpositive() ||
        !std::isfinite(_power_db)
    ) {
        throw std::invalid_argument("Tracking state requires valid observation values and time");
    }

    // Finite components may still produce an unrepresentable total speed.
    static_cast<void>(_velocity.speed());

    if (_errorCovariance)
    {
        if (_errorCovariance->size() != STATE_DIMENSION)
        {
            throw std::invalid_argument("Tracking covariance requires a 9x9 matrix");
        }
        try
        {
            static_cast<void>(math::cholesky(*_errorCovariance));
        }
        catch (const std::domain_error&)
        {
            throw std::invalid_argument("Tracking covariance must be positive definite");
        }
    }
}

} // namespace target
