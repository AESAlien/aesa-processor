#include "track_state.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace target
{
using namespace math::literals;

TrackState::TrackState(
    math::SphericalPosition sphericalPosition,
    math::LinearVelocity dopplerVelocity,
    float power_db,
    std::chrono::milliseconds timestamp,
    math::CartesianPosition position,
    math::CartesianVelocity velocity,
    math::CartesianAcceleration acceleration
) : _sphericalPosition(std::move(sphericalPosition)),
    _dopplerVelocity(dopplerVelocity),
    _power_db(power_db),
    _timestamp(timestamp),
    _position(std::move(position)),
    _velocity(std::move(velocity)),
    _acceleration(std::move(acceleration))
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

void TrackState::validate() const
{
    if (_timestamp.count() < 0 ||
        _sphericalPosition.radius.isNonpositive() ||
        !std::isfinite(_power_db)
    ) {
        throw std::invalid_argument("Tracking state requires valid observation values and time");
    }

    // 각 속도 성분이 유한해도 전체 속력은 표현 범위를 벗어날 수 있다.
    static_cast<void>(_velocity.speed());
}

} // namespace target
