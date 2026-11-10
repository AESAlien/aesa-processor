#include <target/config/tracking_config.hpp>

#include <cmath>
#include <stdexcept>

namespace target
{

TrackingConfig::TrackingConfig(
    math::LinearAcceleration initialAccelerationStandardDeviation,
    math::LinearAcceleration processAccelerationStandardDeviation
) : _initialAccelerationStandardDeviation(initialAccelerationStandardDeviation),
    _processAccelerationStandardDeviation(processAccelerationStandardDeviation)
{
    validate();
}

const math::LinearAcceleration& TrackingConfig::initialAccelerationStandardDeviation() const noexcept
{
    return _initialAccelerationStandardDeviation;
}

const math::LinearAcceleration& TrackingConfig::processAccelerationStandardDeviation() const noexcept
{
    return _processAccelerationStandardDeviation;
}

void TrackingConfig::validate() const
{
    const auto initialSigma = _initialAccelerationStandardDeviation.mps2();
    const auto initialVariance = initialSigma * initialSigma;
    const auto processSigma = _processAccelerationStandardDeviation.mps2();
    const auto processVariance = processSigma * processSigma;
    if (initialSigma <= 0.0 || !std::isfinite(initialVariance) || initialVariance <= 0.0 ||
        processSigma < 0.0 || !std::isfinite(processVariance) ||
        (processSigma > 0.0 && processVariance == 0.0))
    {
        throw std::invalid_argument("Invalid tracking configuration");
    }
}

TrackingConfig TrackingConfig::Builder::build() const
{
    if (!_initialAccelerationStandardDeviation || !_processAccelerationStandardDeviation)
    {
        throw std::logic_error("TrackingConfig::Builder requires every value");
    }

    return TrackingConfig(
        *_initialAccelerationStandardDeviation,
        *_processAccelerationStandardDeviation
    );
}

} // namespace target
