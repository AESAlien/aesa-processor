#include <beam/config/request_beam_config.hpp>

#include <stdexcept>

namespace aesa_processor::beam
{

RequestBeamConfig::RequestBeamConfig() : RequestBeamConfig(Builder().build())
{
}

RequestBeamConfig::RequestBeamConfig(
    aesa_processor::math::Angle confirmationBeamWidth,
    aesa_processor::math::Angle trackingBeamWidth
) : _confirmationBeamWidth(confirmationBeamWidth),
    _trackingBeamWidth(trackingBeamWidth)
{
    validate();
}

aesa_processor::math::Angle RequestBeamConfig::confirmationBeamWidth() const noexcept
{
    return _confirmationBeamWidth;
}

aesa_processor::math::Angle RequestBeamConfig::trackingBeamWidth() const noexcept
{
    return _trackingBeamWidth;
}

void RequestBeamConfig::validate() const
{
    if (_confirmationBeamWidth <= 0_deg || _trackingBeamWidth <= 0_deg)
    {
        throw std::invalid_argument("RequestBeamConfig: beam width must be greater than 0");
    }
}

RequestBeamConfig RequestBeamConfig::Builder::build() const
{
    return RequestBeamConfig(_confirmationBeamWidth, _trackingBeamWidth);
}

} // namespace aesa_processor::beam
