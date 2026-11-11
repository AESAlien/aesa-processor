#pragma once

#include <beam/dto/log/beam_event_logger.hpp>

namespace app
{
// 빔 모듈의 처리 이벤트를 메시지 로그에 기록한다.
// 운용 상태 변경과 자세정보 반영은 info, 자세정보 무시와 요청 거부는 warn으로 기록한다.
class BeamEventMessageLogger final : public beam::BeamEventLogger
{
public:
    static constexpr char LOGGER_NAME[] = "BeamEvent";

    void logOperationStateChanged(beam::OperationState state) noexcept override;
    void logAttitudeApplied(const beam::SetAttitudeCommand& command) noexcept override;
    void logAttitudeIgnoredWhileOff(const beam::SetAttitudeCommand& command) noexcept override;
    void logAttitudeIgnoredAlreadySet(const beam::SetAttitudeCommand& command) noexcept override;
    void logRequestBeamRejected(const beam::RequestBeamResult& result) noexcept override;
};
}
