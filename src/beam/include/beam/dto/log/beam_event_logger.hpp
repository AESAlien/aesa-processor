#pragma once

#include <beam/domain/operation_state.hpp>

namespace beam
{
struct RequestBeamResult;
struct SetAttitudeCommand;

// 빔 모듈의 처리 이벤트를 기록하는 인터페이스. 구현은 빔 모듈 밖(app)에서 주입한다.
// BeamController와 같은 스레드에서 호출되며, 기록 실패가 빔 처리에 영향을 주지 않도록 예외를 던지지 않는다.
class BeamEventLogger
{
public:
    virtual ~BeamEventLogger() = default;

    // 운용 상태가 바뀌었다. 같은 상태의 중복 수신은 호출하지 않는다.
    virtual void logOperationStateChanged(OperationState state) noexcept = 0;

    // 자세정보를 반영했다.
    virtual void logAttitudeApplied(const SetAttitudeCommand& command) noexcept = 0;

    // 운용 상태가 ON이 아니어서 자세정보를 무시했다.
    virtual void logAttitudeIgnoredWhileOff(const SetAttitudeCommand& command) noexcept = 0;

    // 이미 자세정보를 받은 상태여서 무시했다. 자세정보는 ON마다 한 번만 반영한다.
    virtual void logAttitudeIgnoredAlreadySet(const SetAttitudeCommand& command) noexcept = 0;

    // 확인/추적 빔 요청을 버렸다(ACCEPTED가 아님). 잘못된 요청 값의 예외는 호출하지 않는다.
    virtual void logRequestBeamRejected(const RequestBeamResult& result) noexcept = 0;
};
} // namespace beam
