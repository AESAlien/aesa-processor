#include "confirmed_track.hpp"

#include "domain/tracking_algorithm.hpp"

#include <limits>
#include <stdexcept>
#include <utility>

namespace target
{

ConfirmedTrack::ConfirmedTrack(
    std::uint32_t id,
    std::chrono::milliseconds initialTime,
    TrackState state,
    std::shared_ptr<const ITrackingAlgorithm> trackingAlgorithm
) : _id(id),
    _initialTime(initialTime),
    _lastResultTime(state.timestamp()),
    _trackingState(std::make_shared<const TrackState>(std::move(state))),
    _trackingAlgorithm(std::move(trackingAlgorithm))
{
    if (_id == 0 || !_trackingAlgorithm)
    {
        throw std::invalid_argument("Confirmed target requires a nonzero ID and tracking algorithm");
    }
    if (_initialTime.count() < 0 || _lastResultTime <= _initialTime)
    {
        throw std::invalid_argument("Confirmed target requires a state after its nonnegative initial time");
    }
}

std::uint32_t ConfirmedTrack::id() const
{
    return _id;
}

math::Distance ConfirmedTrack::slantRange() const
{
    return sphericalPosition().radius;
}

math::CircularAngle ConfirmedTrack::azimuth() const
{
    return sphericalPosition().direction.azimuth;
}

math::TiltAngle ConfirmedTrack::elevation() const
{
    return sphericalPosition().direction.elevation;
}

const math::SphericalPosition& ConfirmedTrack::sphericalPosition() const
{
    return _trackingState->sphericalPosition();
}

math::LinearVelocity ConfirmedTrack::dopplerVelocity() const
{
    return _trackingState->dopplerVelocity();
}

float ConfirmedTrack::power_db() const
{
    return _trackingState->power_db();
}

std::chrono::milliseconds ConfirmedTrack::initialTime() const
{
    return _initialTime;
}

std::chrono::milliseconds ConfirmedTrack::lastObservationTime() const
{
    return _trackingState->timestamp();
}

std::chrono::milliseconds ConfirmedTrack::lastResultTime() const
{
    return _lastResultTime;
}

std::uint32_t ConfirmedTrack::consecutiveMissCount() const
{
    return _consecutiveMissCount;
}

const std::shared_ptr<const TrackState>& ConfirmedTrack::trackingState() const
{
    return _trackingState;
}

bool ConfirmedTrack::update(const Observation& observation)
{
    const auto time = observation.lastObservationTime();
    if (time <= _lastResultTime || time <= _trackingState->timestamp())
    {
        return false;
    }

    return applyTrackingState(_trackingAlgorithm->update(*this, observation), time);
}

bool ConfirmedTrack::applyTrackingState(
    std::optional<TrackState> state,
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
    auto nextState = std::make_shared<const TrackState>(std::move(*state));
    _trackingState = std::move(nextState);
    _lastResultTime = time;
    _consecutiveMissCount = 0;
    return true;
}

bool ConfirmedTrack::recordTrackingMiss(std::chrono::milliseconds time)
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

std::optional<TargetPrediction> ConfirmedTrack::predict(std::chrono::milliseconds time) const
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

} // namespace target
