#include "target_config.hpp"

#include <cmath>
#include <stdexcept>

namespace target
{

using namespace math::literals;

TargetConfig::TargetConfig()
    : maxAssociationSpeed(1000.0_mps)
    , maxInitialDopplerDifference(30.0_mps)
    , measurementStandardDeviation(7.0_m)
    , positionNdsThreshold(11.345)
    , confirmationDelay(100)
    , trackingInterval(1000)
    , maxConsecutiveTrackingMisses(3)
{
    validate();
}

void TargetConfig::validate() const
{
    const double sigma = measurementStandardDeviation.m();
    const double variance = 2.0 * sigma * sigma;
    if (maxAssociationSpeed.lessThan(0.0_mps, math::LinearVelocity::NONE_TOLERANCE)
        || maxInitialDopplerDifference.lessThan(0.0_mps, math::LinearVelocity::NONE_TOLERANCE)
        || sigma <= 0.0 || !std::isfinite(variance) || variance <= 0.0
        || !std::isfinite(positionNdsThreshold) || positionNdsThreshold < 0.0
        || confirmationDelay.count() <= 0 || trackingInterval.count() <= 0
        || maxConsecutiveTrackingMisses == 0)
    {
        throw std::invalid_argument("Invalid Target configuration");
    }
}

} // namespace target
