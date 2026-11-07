#include <target/config/measurement_config.hpp>

#include <cmath>
#include <stdexcept>

namespace target
{

MeasurementConfig::MeasurementConfig(
    math::Length standardDeviation
) : _standardDeviation(standardDeviation)
{
    validate();
}

const math::Length& MeasurementConfig::standardDeviation() const noexcept
{
    return _standardDeviation;
}

void MeasurementConfig::validate() const
{
    const double sigma = _standardDeviation.m();
    const double variance = sigma * sigma;
    const double differenceVariance = 2.0 * variance;
    if (sigma <= 0.0 || !std::isfinite(variance) || variance <= 0.0 ||
        !std::isfinite(differenceVariance))
    {
        throw std::invalid_argument("Invalid measurement configuration");
    }
}

MeasurementConfig MeasurementConfig::Builder::build() const
{
    if (!_standardDeviation)
    {
        throw std::logic_error("MeasurementConfig::Builder requires every value");
    }

    return MeasurementConfig(
        *_standardDeviation
    );
}

} // namespace target
