#include "target.hpp"

#include "calculation/tracking/tracking_algorithm.hpp"
#include "validation/detection_validation.hpp"

#include <limits>
#include <stdexcept>
#include <utility>

namespace target
{

Target::Target(
    math::Distance slantRange,
    math::Angle azimuth,
    math::Angle elevation,
    math::Velocity dopplerVelocity,
    float power_db,
    std::chrono::milliseconds time,
    std::shared_ptr<const ITrackingAlgorithm> trackingAlgorithm)
    : _initialSlantRange(slantRange)
    , _initialAzimuth(azimuth)
    , _initialElevation(elevation)
    , _initialDopplerVelocity(dopplerVelocity)
    , _initialPower_db(power_db)
    , _initialTime(time)
    , _lastResultTime(time)
    , _trackingAlgorithm(std::move(trackingAlgorithm))
{
    if (!_trackingAlgorithm)
    {
        throw std::invalid_argument("Tracking algorithm must not be null");
    }

    if (time.count() < 0
        || !validation::hasValidMeasurement(slantRange, azimuth, elevation, dopplerVelocity))
    {
        throw std::invalid_argument("Target requires a valid observation and time");
    }
}

std::uint32_t Target::id() const
{
    return _id;
}

TargetState Target::state() const
{
    if (_id == 0)
    {
        return TargetState::UNREGISTERED;
    }
    return _trackingState ? TargetState::TRACKING : TargetState::TENTATIVE;
}

math::Distance Target::slantRange() const
{
    return _trackingState ? _trackingState->slantRange : _initialSlantRange;
}

math::Angle Target::azimuth() const
{
    return _trackingState ? _trackingState->azimuth : _initialAzimuth;
}

math::Angle Target::elevation() const
{
    return _trackingState ? _trackingState->elevation : _initialElevation;
}

math::Velocity Target::dopplerVelocity() const
{
    return _trackingState ? _trackingState->dopplerVelocity : _initialDopplerVelocity;
}

float Target::power_db() const
{
    return _trackingState ? _trackingState->power_db : _initialPower_db;
}

std::chrono::milliseconds Target::initialTime() const
{
    return _initialTime;
}

std::chrono::milliseconds Target::lastObservationTime() const
{
    return _trackingState ? _trackingState->timestamp : _initialTime;
}

std::chrono::milliseconds Target::lastResultTime() const
{
    return _lastResultTime;
}

std::uint32_t Target::consecutiveMissCount() const
{
    return _consecutiveMissCount;
}

const std::optional<TargetTrackingState>& Target::trackingState() const
{
    return _trackingState;
}

void Target::registerAs(std::uint32_t id)
{
    if (id == 0 || state() != TargetState::UNREGISTERED || _trackingState)
    {
        throw std::invalid_argument("Only an unregistered Target can receive a nonzero ID");
    }
    _id = id;
}

bool Target::confirm(const Target& observation)
{
    const auto time = observation.lastObservationTime();
    if (state() != TargetState::TENTATIVE
        || observation.state() != TargetState::UNREGISTERED
        || !validation::isValidTarget(observation)
        || time <= _lastResultTime || time <= _initialTime)
    {
        return false;
    }

    const auto initialized = _trackingAlgorithm->initialize(*this, observation);
    if (!initialized || initialized->timestamp != time
        || !validation::isValidTrackingState(*initialized))
    {
        return false;
    }

    _trackingState = *initialized;
    _lastResultTime = time;
    _consecutiveMissCount = 0;
    return true;
}

bool Target::update(const Target& observation)
{
    const auto time = observation.lastObservationTime();
    if (state() != TargetState::TRACKING
        || observation.state() != TargetState::UNREGISTERED
        || !validation::isValidTarget(observation)
        || time <= _lastResultTime || time <= _trackingState->timestamp)
    {
        return false;
    }

    const auto updated = _trackingAlgorithm->update(*this, observation);
    if (!updated || updated->timestamp != time
        || !validation::isValidTrackingState(*updated))
    {
        return false;
    }

    _trackingState = *updated;
    _lastResultTime = time;
    _consecutiveMissCount = 0;
    return true;
}

bool Target::recordTrackingMiss(std::chrono::milliseconds time)
{
    if (state() != TargetState::TRACKING || !markResultProcessed(time))
    {
        return false;
    }

    if (_consecutiveMissCount < std::numeric_limits<std::uint32_t>::max())
    {
        ++_consecutiveMissCount;
    }
    return true;
}

bool Target::markResultProcessed(std::chrono::milliseconds time)
{
    if (time.count() < 0 || time <= _lastResultTime)
    {
        return false;
    }

    _lastResultTime = time;
    return true;
}

std::optional<TargetPrediction> Target::predict(std::chrono::milliseconds time) const
{
    if (state() != TargetState::TRACKING || time < _trackingState->timestamp)
    {
        return std::nullopt;
    }

    const auto prediction = _trackingAlgorithm->predict(*this, time);
    if (!prediction || prediction->timestamp != time
        || !validation::isValidPrediction(*prediction))
    {
        return std::nullopt;
    }
    return prediction;
}

} // namespace target
