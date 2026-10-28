#include <beam/config/request_beam_config.hpp>

namespace beam
{

RequestBeamConfig::RequestBeamConfig(
    BeamWidth confirmationBeamWidth,
    BeamWidth trackingBeamWidth
) : confirmationBeamWidth(confirmationBeamWidth),
    trackingBeamWidth(trackingBeamWidth)
{
}

RequestBeamConfig RequestBeamConfig::Builder::build() const
{
    if (!_confirmationBeamWidth || !_trackingBeamWidth)
    {
        throw std::logic_error("RequestBeamConfig::Builder requires every value");
    }

    return RequestBeamConfig(*_confirmationBeamWidth, *_trackingBeamWidth);
}

} // namespace beam
