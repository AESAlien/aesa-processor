#include <target/config/target_config.hpp>

#include <cmath>
#include <stdexcept>

namespace target
{

using namespace math::literals;

TargetConfig::TargetConfig(
    math::LinearVelocity maxAssociationSpeed,
    math::LinearVelocity maxInitialDopplerDifference,
    math::Length measurementStandardDeviation,
    double positionNdsThreshold,
    std::chrono::milliseconds confirmationDelay,
    std::chrono::milliseconds trackingInterval,
    std::uint32_t maxConsecutiveTrackingMisses,
    double initialAccelerationStandardDeviation,
    std::size_t maxReportedTrackCount)
    : _maxAssociationSpeed(maxAssociationSpeed)
    , _maxInitialDopplerDifference(maxInitialDopplerDifference)
    , _measurementStandardDeviation(measurementStandardDeviation)
    , _positionNdsThreshold(positionNdsThreshold)
    , _confirmationDelay(confirmationDelay)
    , _trackingInterval(trackingInterval)
    , _maxConsecutiveTrackingMisses(maxConsecutiveTrackingMisses)
    , _initialAccelerationStandardDeviation(initialAccelerationStandardDeviation)
    , _maxReportedTrackCount(maxReportedTrackCount)
{
    validate();
}

const math::LinearVelocity& TargetConfig::maxAssociationSpeed() const noexcept
{
    return _maxAssociationSpeed;
}

const math::LinearVelocity& TargetConfig::maxInitialDopplerDifference() const noexcept
{
    return _maxInitialDopplerDifference;
}

const math::Length& TargetConfig::measurementStandardDeviation() const noexcept
{
    return _measurementStandardDeviation;
}

double TargetConfig::positionNdsThreshold() const noexcept
{
    return _positionNdsThreshold;
}

std::chrono::milliseconds TargetConfig::confirmationDelay() const noexcept
{
    return _confirmationDelay;
}

std::chrono::milliseconds TargetConfig::trackingInterval() const noexcept
{
    return _trackingInterval;
}

std::uint32_t TargetConfig::maxConsecutiveTrackingMisses() const noexcept
{
    return _maxConsecutiveTrackingMisses;
}

double TargetConfig::initialAccelerationStandardDeviation() const noexcept
{
    return _initialAccelerationStandardDeviation;
}

std::size_t TargetConfig::maxReportedTrackCount() const noexcept
{
    return _maxReportedTrackCount;
}

void TargetConfig::validate() const
{
    const double sigma = _measurementStandardDeviation.m();
    const double variance = sigma * sigma;
    const double differenceVariance = 2.0 * variance;
    const double accelerationVariance = _initialAccelerationStandardDeviation
        * _initialAccelerationStandardDeviation;
    if (_maxAssociationSpeed.lessThan(0.0_mps, math::LinearVelocity::NONE_TOLERANCE)
        || _maxInitialDopplerDifference.lessThan(0.0_mps, math::LinearVelocity::NONE_TOLERANCE)
        || sigma <= 0.0 || !std::isfinite(variance) || variance <= 0.0
        || !std::isfinite(differenceVariance)
        || !std::isfinite(_positionNdsThreshold) || _positionNdsThreshold < 0.0
        || _confirmationDelay.count() <= 0 || _trackingInterval.count() <= 0
        || _maxConsecutiveTrackingMisses == 0
        || !std::isfinite(_initialAccelerationStandardDeviation)
        || _initialAccelerationStandardDeviation <= 0.0
        || !std::isfinite(accelerationVariance) || accelerationVariance <= 0.0
        || _maxReportedTrackCount == 0 || _maxReportedTrackCount > MAXIMUM_REPORTED_TRACK_COUNT)
    {
        throw std::invalid_argument("Invalid Target configuration");
    }
}

} // namespace target
