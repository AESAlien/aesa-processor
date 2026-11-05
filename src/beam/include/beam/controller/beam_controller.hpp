#pragma once

#include <beam/config/beam_config.hpp>
#include <beam/dto/request_beam_command.hpp>
#include <beam/dto/request_beam_result.hpp>
#include <beam/dto/set_attitude_command.hpp>
#include <beam/dto/set_operation_state_command.hpp>
#include <beam/dto/transmit_beam_command.hpp>

#include <chrono>
#include <memory>
#include <optional>

namespace beam
{

class BeamRequestFactory;
class BeamScheduler;

// 빔 모듈의 외부 인터페이스. 내부 스케줄러에 호출을 전달한다.
// 스레드 안전하지 않으므로 모든 메서드를 같은 스레드에서 호출해야 한다.
class BeamController
{
public:
    // 주어진 설정으로 만든다. 설정은 BeamConfig 생성 시 이미 검증되어 있다.
    explicit BeamController(const BeamConfig& config);
    ~BeamController();

    BeamController(BeamController&&) noexcept;
    BeamController& operator=(BeamController&&) noexcept;

    void setOperationState(SetOperationStateCommand command);

    // 자세정보가 잘못되면(roll/yaw 범위 밖, 짐벌락) 운용 상태와 관계없이 std::invalid_argument를 던지며, 이때 상태는 변하지 않는다.
    // 위도·경도·고도는 쓰지 않으므로 검사하지 않는다.
    // 올바른 자세정보는 ON이고 아직 자세정보를 받지 않았을 때만 반영되고(탐색 격자와 추적 영역 판정에 쓰인다), 그 외에는 무시된다.
    void setAttitude(SetAttitudeCommand command);

    // 요청 값이 잘못되면(알 수 없는 종류, 음수 transmitTime, requestId 0) 운용 상태와 관계없이 std::invalid_argument를 던진다.
    // 생성자에 주입한 요청 빔 설정(빔 폭)은 BeamRequestFactory가 요청을 만들 때 반영된다.
    // 올바른 요청의 처리 결과를 반환한다. 결과에는 상태와 함께 요청의 requestId가 그대로 담긴다.
    // - NOT_READY: 준비 상태(ON이고 자세정보를 받음)가 아니어서 버렸다.
    // - OUT_OF_COVERAGE: 요청 방향(ENU)을 안테나 기준으로 바꾼 값이 추적 영역(RequestCoverageConfig) 밖이어서 버렸다.
    // - ACCEPTED: 큐에 넣었다.
    // 버려진 요청은 송신되지 않으므로, 표적을 어떻게 할지는 호출한 쪽이 결과를 보고 정한다.
    RequestBeamResult requestBeam(RequestBeamCommand command);

    // 송신할 빔이 없으면 std::nullopt를 반환한다. 틱마다 한 번 호출한다.
    std::optional<TransmitBeamCommand> nextBeamCommand(std::chrono::milliseconds currentTime);

private:
    // 자세정보를 받을 때 탐색 격자와 추적 영역 판정을 만드는 데 쓴다.
    BeamConfig _config;
    std::unique_ptr<BeamScheduler> _scheduler;
    // RequestBeamCommand를 BeamRequest로 바꾼다. 요청 빔 설정은 생성 시 한 번 주입된다.
    std::unique_ptr<BeamRequestFactory> _requestFactory;
};

} // namespace beam
