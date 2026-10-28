#include "scheduler/beam_request_factory.hpp"

#include <stdexcept>

namespace beam
{
namespace
{

// 송신 시각이 같은 요청 중 값이 클수록 먼저 보낸다. 확인 빔을 추적 빔보다 먼저 보낸다.
constexpr int CONFIRMATION_PRIORITY = 1;
constexpr int TRACKING_PRIORITY = 0;

} // namespace

BeamRequestFactory::BeamRequestFactory(const RequestBeamConfig& config) : _config(config)
{
}

BeamRequest BeamRequestFactory::create(const RequestBeamCommand& command) const
{
    // 잘못된 transmitTime/requestId이면 BeamRequest 생성자가 예외를 던진다.
    switch (command.beamType)
    {
    case RequestBeamType::CONFIRMATION:
        return BeamRequest(
            BeamType::CONFIRMATION,
            _config.confirmationBeamWidth,
            CONFIRMATION_PRIORITY,
            command.transmitTime,
            command.requestId,
            command.direction_ant);

    case RequestBeamType::TRACKING:
        return BeamRequest(
            BeamType::TRACKING,
            _config.trackingBeamWidth,
            TRACKING_PRIORITY,
            command.transmitTime,
            command.requestId,
            command.direction_ant);
    }

    throw std::invalid_argument("BeamRequestFactory: unknown request beam type");
}

} // namespace beam
