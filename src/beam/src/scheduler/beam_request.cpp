#include "scheduler/beam_request.hpp"

#include <stdexcept>
#include <utility>

namespace beam
{

BeamRequest::BeamRequest(
    std::shared_ptr<const IRequestBeamPolicy> policy,
    std::chrono::milliseconds transmitTime,
    std::uint32_t requestId,
    math::SphericalDirection direction_enu
) : _policy(std::move(policy)),
    _transmitTime(transmitTime),
    _requestId(requestId),
    _direction_enu(direction_enu)
{
    validate();
}

BeamType BeamRequest::transmitBeamType() const
{
    return _policy->transmitBeamType();
}

BeamWidth BeamRequest::beamWidth() const
{
    return _policy->beamWidth();
}

int BeamRequest::priority() const
{
    return _policy->priority();
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
    if (_policy == nullptr)
    {
        throw std::invalid_argument("BeamRequest: policy must not be null");
    }

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
