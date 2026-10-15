#include <beam/config/request_beam_config.hpp>

#include <stdexcept>

namespace beam
{
namespace
{
using namespace math::literals;

void validate(math::Angle confirmationBeamWidth, math::Angle trackingBeamWidth)
{
    if (confirmationBeamWidth <= 0_deg || trackingBeamWidth <= 0_deg)
    {
        throw std::invalid_argument("RequestBeamConfig: beam width must be greater than 0");
    }
}
} // namespace

RequestBeamConfig::RequestBeamConfig() : RequestBeamConfig(Builder().build())
{
}

RequestBeamConfig::RequestBeamConfig(
    math::Angle confirmationBeamWidth,
    math::Angle trackingBeamWidth
) : _confirmationBeamWidth(confirmationBeamWidth),
    _trackingBeamWidth(trackingBeamWidth)
{
    validate(_confirmationBeamWidth, _trackingBeamWidth);
}

math::Angle RequestBeamConfig::confirmationBeamWidth() const noexcept
{
    return _confirmationBeamWidth;
}

math::Angle RequestBeamConfig::trackingBeamWidth() const noexcept
{
    return _trackingBeamWidth;
}

RequestBeamConfig RequestBeamConfig::Builder::build() const
{
    return RequestBeamConfig(_confirmationBeamWidth, _trackingBeamWidth);
}

} // namespace beam
