#include "scheduler/beam_request.hpp"

#include <stdexcept>

namespace beam
{

BeamRequest::BeamRequest(
    BeamType transmitBeamType,
    math::Angle beamWidth,
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

math::Angle BeamRequest::beamWidth() const noexcept
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
        throw std::invalid_argument("BeamRequest: transmit time must not be negative");
    }

    if (_requestId == 0)
    {
        throw std::invalid_argument("BeamRequest: request id must not be 0");
    }
}

} // namespace beam
