#include "target.hpp"

#include "calculation/tracking/tracking_algorithm.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace target
{
namespace
{

constexpr double MAX_INITIAL_SPEED_MPS = 1000.0;
constexpr double MAX_INITIAL_DOPPLER_DIFFERENCE_MPS = 30.0;

bool isValidPlot(const DetectionEvent::Plot& plot)
{
    return std::isfinite(plot.slantRange.m()) && plot.slantRange.m() > 0.0
        && std::isfinite(plot.azimuth.deg())
        && std::isfinite(plot.elevation.deg())
        && std::abs(plot.elevation.deg()) <= 90.0
        && std::isfinite(plot.dopplerVelocity.mps());
}

} // namespace

Target::Target(
    std::uint32_t id,
    const DetectionEvent::Plot& initialPlot,
    std::chrono::milliseconds initialTime,
    std::shared_ptr<const ITrackingAlgorithm> trackingAlgorithm)
    : _id(id)
    , _initialSlantRange(initialPlot.slantRange)
    , _initialAzimuth(initialPlot.azimuth)
    , _initialElevation(initialPlot.elevation)
    , _initialDopplerVelocity(initialPlot.dopplerVelocity)
    , _initialPowerDb(initialPlot.power_db)
    , _initialTime(initialTime)
    , _lastResultTime(initialTime)
    , _trackingAlgorithm(std::move(trackingAlgorithm))
{
    if (!_trackingAlgorithm)
    {
        throw std::invalid_argument("Tracking algorithm must not be null");
    }

    if (_id == 0 || initialTime.count() < 0 || !isValidPlot(initialPlot))
    {
        throw std::invalid_argument("Target requires a nonzero ID and valid initial detection/time");
    }
}

std::uint32_t Target::id() const
{
    return _id;
}

TargetState Target::state() const
{
    return _trackingState ? TargetState::TRACKING : TargetState::TENTATIVE;
}

DetectionEvent::Plot Target::initialPlot() const
{
    return DetectionEvent::Plot{_initialSlantRange, _initialAzimuth, _initialElevation,
        _initialDopplerVelocity, _initialPowerDb};
}

DetectionEvent::Plot Target::lastPlot() const
{
    if (!_trackingState)
    {
        return initialPlot();
    }

    return DetectionEvent::Plot{_trackingState->slantRange, _trackingState->azimuth,
        _trackingState->elevation, _trackingState->dopplerVelocity, _trackingState->powerDb};
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

bool Target::canConfirm(const DetectionEvent::Plot& plot, std::chrono::milliseconds time) const
{
    return initializeTrackingState(plot, time).has_value();
}

bool Target::confirm(const DetectionEvent::Plot& plot, std::chrono::milliseconds time)
{
    auto initialized = initializeTrackingState(plot, time);
    if (!initialized)
    {
        return false;
    }

    _trackingState = *initialized;
    _lastResultTime = time;
    _consecutiveMissCount = 0;
    return true;
}

bool Target::update(const DetectionEvent::Plot& plot, std::chrono::milliseconds time)
{
    if (!_trackingState || !isValidPlot(plot)
        || time <= _lastResultTime || time <= _trackingState->timestamp)
    {
        return false;
    }

    auto updated = _trackingAlgorithm->update(*_trackingState, plot, time);
    if (!updated || updated->timestamp != time)
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
    if (!_trackingState || !markResultProcessed(time))
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
    if (!_trackingState || time < _trackingState->timestamp)
    {
        return std::nullopt;
    }

    const auto prediction = _trackingAlgorithm->predict(*_trackingState, time);
    if (!prediction || prediction->timestamp != time)
    {
        return std::nullopt;
    }
    return prediction;
}

std::optional<TargetTrackingState> Target::initializeTrackingState(
    const DetectionEvent::Plot& plot,
    std::chrono::milliseconds time) const
{
    if (state() != TargetState::TENTATIVE || !isValidPlot(plot)
        || time <= _lastResultTime || time <= _initialTime)
    {
        return std::nullopt;
    }

    auto initialized = _trackingAlgorithm->initialize(initialPlot(), _initialTime, plot, time);
    if (!initialized || initialized->timestamp != time)
    {
        return std::nullopt;
    }

    const auto& velocity = initialized->velocityMetersPerSecond;
    const double speed = std::hypot(velocity[0], velocity[1], velocity[2]);
    const double dopplerDifference = std::abs(plot.dopplerVelocity.mps()
        - _initialDopplerVelocity.mps());
    if (!std::isfinite(speed) || speed > MAX_INITIAL_SPEED_MPS
        || !std::isfinite(dopplerDifference)
        || dopplerDifference > MAX_INITIAL_DOPPLER_DIFFERENCE_MPS)
    {
        return std::nullopt;
    }

    return initialized;
}

} // namespace target
