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
) : confirmationBeamWidth(confirmationBeamWidth),
    trackingBeamWidth(trackingBeamWidth)
{
    validate();
}

void RequestBeamConfig::validate() const
{
    if (confirmationBeamWidth <= 0_deg || trackingBeamWidth <= 0_deg)
    {
        throw std::invalid_argument("RequestBeamConfig: beam width must be greater than 0");
    }
}

RequestBeamConfig RequestBeamConfig::Builder::build() const
{
    return RequestBeamConfig(_confirmationBeamWidth, _trackingBeamWidth);
}

} // namespace beam
