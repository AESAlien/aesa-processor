#include "scheduler/beam_request.hpp"

#include "scheduler/request_beam_policy.hpp"

#include <stdexcept>

namespace beam
{

BeamRequest::BeamRequest(
    RequestBeamType beamType,
    std::chrono::milliseconds transmitTime,
    std::uint32_t requestId,
    math::SphericalDirection direction_ant
) : _beamType(beamType),
    _transmitTime(transmitTime),
    _requestId(requestId),
    _direction_ant(direction_ant)
{
    validate();
}

BeamRequest::BeamRequest(const RequestBeamCommand& command)
    : BeamRequest(command.beamType, command.transmitTime, command.requestId, command.direction_ant)
{
}

RequestBeamType BeamRequest::beamType() const noexcept
{
    return _beamType;
}

std::chrono::milliseconds BeamRequest::transmitTime() const noexcept
{
    return _transmitTime;
}

std::uint32_t BeamRequest::requestId() const noexcept
{
    return _requestId;
}

const math::SphericalDirection& BeamRequest::direction_ant() const noexcept
{
    return _direction_ant;
}

void BeamRequest::validate() const
{
    // 알 수 없는 종류면 requestBeamPolicyOf가 std::invalid_argument를 던진다.
    (void)requestBeamPolicyOf(_beamType);

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
