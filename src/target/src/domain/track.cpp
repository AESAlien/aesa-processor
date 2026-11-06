#include "track.hpp"

#include "domain/tracking_algorithm.hpp"
#include "math/trigonometry.hpp"

#include <limits>
#include <stdexcept>
#include <utility>

namespace target
{

Track::Track(
    std::uint32_t id,
    std::chrono::milliseconds initialTime,
    TargetTrackingState state,
    std::shared_ptr<const ITrackingAlgorithm> trackingAlgorithm
) : _id(id),
    _initialTime(initialTime),
    _lastResultTime(state.timestamp()),
    _trackingState(std::make_shared<const TargetTrackingState>(std::move(state))),
    _trackingAlgorithm(std::move(trackingAlgorithm))
{
}

std::uint32_t Track::id() const
{
    return _id;
}

math::Distance Track::slantRange() const
{
    return sphericalPosition().radius;
}

math::CircularAngle Track::azimuth() const
{
    return sphericalPosition().direction.azimuth;
}

math::TiltAngle Track::elevation() const
{
    return sphericalPosition().direction.elevation;
}

const math::SphericalPosition& Track::sphericalPosition() const
{
    return _trackingState->sphericalPosition();
}

math::LinearVelocity Track::dopplerVelocity() const
{
    return _trackingState->dopplerVelocity();
}

float Track::power_db() const
{
    return _trackingState->power_db();
}

std::chrono::milliseconds Track::initialTime() const
{
    return _initialTime;
}

std::chrono::milliseconds Track::lastObservationTime() const
{
    return _trackingState->timestamp();
}

std::chrono::milliseconds Track::lastResultTime() const
{
    return _lastResultTime;
}

std::uint32_t Track::consecutiveMissCount() const
{
    return _consecutiveMissCount;
}

const std::shared_ptr<const TargetTrackingState>& Track::trackingState() const
{
    return _trackingState;
}

bool Track::update(const Observation& observation)
{
    const auto time = observation.lastObservationTime();
    if (time <= _lastResultTime || time <= _trackingState->timestamp())
    {
        return false;
    }

    return applyTrackingState(_trackingAlgorithm->update(*this, observation), time);
}

bool Track::applyTrackingState(
    std::optional<TargetTrackingState> state,
    std::chrono::milliseconds time
) {
    if (!state)
    {
        return false;
    }
    if (state->timestamp() != time)
    {
        throw std::invalid_argument("Tracking state must use the observation time");
    }

    // Finish allocations before replacing the previous immutable state.
    auto nextState = std::make_shared<const TargetTrackingState>(std::move(*state));
    _trackingState = std::move(nextState);
    _lastResultTime = time;
    _consecutiveMissCount = 0;
    return true;
}

bool Track::recordTrackingMiss(std::chrono::milliseconds time)
{
    if (time <= _lastResultTime)
    {
        return false;
    }
    _lastResultTime = time;
    if (_consecutiveMissCount < std::numeric_limits<std::uint32_t>::max())
    {
        ++_consecutiveMissCount;
    }
    return true;
}

std::optional<TargetPrediction> Track::predict(std::chrono::milliseconds time) const
{
    if (time < _trackingState->timestamp())
    {
        return std::nullopt;
    }

    const auto prediction = _trackingAlgorithm->predict(*this, time);
    if (!prediction)
    {
        return std::nullopt;
    }
    if (prediction->timestamp() != time)
    {
        throw std::invalid_argument("Prediction must use the requested time");
    }
    return prediction;
}

TrackEvent Track::toTrackEvent() const
{
    if (id() > std::numeric_limits<std::uint16_t>::max())
    {
        throw std::overflow_error("Target ID cannot be represented by TrackEvent");
    }

    const auto& state = *trackingState();
    using namespace math::literals;

    const auto& position = state.position();
    const auto& velocity = state.velocity();
    const auto horizontalRange = position.horizontalDistance();
    const auto horizontalSpeed = velocity.horizontalSpeed();
    const auto speed = velocity.speed();
    const math::CircularAngle heading(horizontalSpeed.greaterThan(0.0_mps, math::LinearVelocity::NONE_TOLERANCE)
        ? math::atan2(velocity.x, velocity.y)
        : 0_deg);
    const math::TiltAngle flightPathAngle(speed.greaterThan(0.0_mps, math::LinearVelocity::NONE_TOLERANCE)
        ? math::atan2(velocity.z, horizontalSpeed)
        : 0_deg);
    return TrackEvent(
        static_cast<std::uint16_t>(id()),
        TrackEvent::TrackState::TRACKING,
        slantRange(),
        horizontalRange,
        azimuth(),
        elevation(),
        dopplerVelocity(),
        math::TiltAngle(0_deg),
        math::CircularAngle(0_deg),
        position.z,
        speed,
        heading,
        flightPathAngle,
        lastObservationTime());
}

} // namespace target
