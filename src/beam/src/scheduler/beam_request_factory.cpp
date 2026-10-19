#include "scheduler/beam_request_factory.hpp"

namespace beam
{

BeamRequestFactory::BeamRequestFactory(const RequestBeamConfig& config) : _policyFactory(config)
{
}

BeamRequest BeamRequestFactory::create(const RequestBeamCommand& command) const
{
    // 알 수 없는 종류이면 정책 팩토리가, 잘못된 transmitTime/requestId이면 BeamRequest 생성자가 예외를 던진다.
    return BeamRequest(
        _policyFactory.create(command.beamType),
        command.transmitTime,
        command.requestId,
        command.direction_ant);
}

} // namespace beam
