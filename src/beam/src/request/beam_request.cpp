#include "request/beam_request.hpp"

#include <sstream>
#include <stdexcept>

namespace beam
{

BeamRequest::BeamRequest(
    BeamType transmitBeamType,
    BeamWidth beamWidth,
    int priority,
    std::chrono::milliseconds transmitTime,
    std::uint32_t requestId,
    math::SphericalDirection direction_enu
) : _transmitBeamType(transmitBeamType),
    _beamWidth(beamWidth),
    _priority(priority),
    _transmitTime(transmitTime),
    _requestId(requestId),
    _direction_enu(direction_enu)
{
    validate();
}

BeamType BeamRequest::transmitBeamType() const noexcept
{
    return _transmitBeamType;
}

BeamWidth BeamRequest::beamWidth() const noexcept
{
    return _beamWidth;
}

int BeamRequest::priority() const noexcept
{
    return _priority;
}

std::chrono::milliseconds BeamRequest::transmitTime() const noexcept
{
    return _transmitTime;
}

std::uint32_t BeamRequest::requestId() const noexcept
{
    return _requestId;
}

const math::SphericalDirection& BeamRequest::direction_enu() const noexcept
{
    return _direction_enu;
}

void BeamRequest::validate() const
{
    if (_transmitTime.count() < 0)
    {
        std::ostringstream message;
        message << "BeamRequest: transmit time must not be negative"
                << ": transmitTime=" << _transmitTime.count() << "ms"
                << ", requestId=" << _requestId;
        throw std::invalid_argument(message.str());
    }

    if (_requestId == 0)
    {
        std::ostringstream message;
        message << "BeamRequest: request id must not be 0"
                << ": transmitTime=" << _transmitTime.count() << "ms";
        throw std::invalid_argument(message.str());
    }
}

} // namespace beam
