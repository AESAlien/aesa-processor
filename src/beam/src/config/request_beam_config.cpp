#include <beam/config/request_beam_config.hpp>

#include <stdexcept>

namespace beam
{

RequestBeamConfig::RequestBeamConfig() : RequestBeamConfig(Builder().build())
{
}

RequestBeamConfig::RequestBeamConfig(
    math::Angle confirmationBeamWidth,
    math::Angle trackingBeamWidth
) : _confirmationBeamWidth(confirmationBeamWidth),
    _trackingBeamWidth(trackingBeamWidth)
{
    validate();
}

math::Angle RequestBeamConfig::confirmationBeamWidth() const noexcept
{
    return _confirmationBeamWidth;
}

math::Angle RequestBeamConfig::trackingBeamWidth() const noexcept
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

} // namespace beam
