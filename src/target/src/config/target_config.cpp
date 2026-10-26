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
    : maxAssociationSpeed(maxAssociationSpeed)
    , maxInitialDopplerDifference(maxInitialDopplerDifference)
    , measurementStandardDeviation(measurementStandardDeviation)
    , positionNdsThreshold(positionNdsThreshold)
    , confirmationDelay(confirmationDelay)
    , trackingInterval(trackingInterval)
    , maxConsecutiveTrackingMisses(maxConsecutiveTrackingMisses)
    , initialAccelerationStandardDeviation(initialAccelerationStandardDeviation)
    , maxReportedTrackCount(maxReportedTrackCount)
{
    validate();
}

void TargetConfig::validate() const
{
    const double sigma = measurementStandardDeviation.m();
    const double variance = sigma * sigma;
    const double differenceVariance = 2.0 * variance;
    const double accelerationVariance = initialAccelerationStandardDeviation
        * initialAccelerationStandardDeviation;
    if (maxAssociationSpeed.lessThan(0.0_mps, math::LinearVelocity::NONE_TOLERANCE)
        || maxInitialDopplerDifference.lessThan(0.0_mps, math::LinearVelocity::NONE_TOLERANCE)
        || sigma <= 0.0 || !std::isfinite(variance) || variance <= 0.0
        || !std::isfinite(differenceVariance)
        || !std::isfinite(positionNdsThreshold) || positionNdsThreshold < 0.0
        || confirmationDelay.count() <= 0 || trackingInterval.count() <= 0
        || maxConsecutiveTrackingMisses == 0
        || !std::isfinite(initialAccelerationStandardDeviation)
        || initialAccelerationStandardDeviation <= 0.0
        || !std::isfinite(accelerationVariance) || accelerationVariance <= 0.0
        || maxReportedTrackCount == 0 || maxReportedTrackCount > MAXIMUM_REPORTED_TRACK_COUNT)
    {
        throw std::invalid_argument("Invalid Target configuration");
    }
}

} // namespace target
