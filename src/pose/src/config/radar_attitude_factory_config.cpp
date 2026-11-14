#include <pose/config/radar_attitude_factory_config.hpp>

#include <stdexcept>

namespace pose
{

using namespace math::literals;

RadarAttitudeFactoryConfig::RadarAttitudeFactoryConfig(math::Angle maxAbsPitch)
    : _maxAbsPitch(maxAbsPitch)
{
    validate();
}

const math::Angle& RadarAttitudeFactoryConfig::maxAbsPitch() const noexcept
{
    return _maxAbsPitch;
}

void RadarAttitudeFactoryConfig::validate() const
{
    if (_maxAbsPitch.lessThanOrEqual(0_deg, math::Angle::NONE_TOLERANCE) ||
        _maxAbsPitch.greaterThan(90_deg, math::Angle::NONE_TOLERANCE))
    {
        throw std::invalid_argument("RadarAttitudeFactoryConfig: maxAbsPitch must be in (0, 90] degrees");
    }
}

RadarAttitudeFactoryConfig::Builder& RadarAttitudeFactoryConfig::Builder::maxAbsPitch(math::Angle value)
{
    _maxAbsPitch = value;
    return *this;
}

RadarAttitudeFactoryConfig RadarAttitudeFactoryConfig::Builder::build() const
{
    if (!_maxAbsPitch)
    {
        throw std::logic_error("RadarAttitudeFactoryConfig::Builder requires every value");
    }
    return RadarAttitudeFactoryConfig(*_maxAbsPitch);
}

} // namespace pose
