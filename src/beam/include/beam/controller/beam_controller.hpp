#pragma once

#include <beam/config/beam_config.hpp>
#include <beam/dto/request_beam_command.hpp>
#include <beam/dto/set_attitude_command.hpp>
#include <beam/dto/set_operation_state_command.hpp>
#include <beam/dto/transmit_beam_command.hpp>

#include <chrono>
#include <memory>
#include <optional>

namespace beam
{

class BeamRequestFactory;
class Scheduler;

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

    // 자세정보가 잘못되면(roll/yaw 범위 밖, 짐벌락) 운용 상태와 관계없이 std::invalid_argument를 던진다.
    // 위도·경도·고도는 쓰지 않으므로 검사하지 않는다.
    void setAttitude(SetAttitudeCommand command);

    // 요청 값이 잘못되면(알 수 없는 종류, 음수 transmitTime, requestId 0) 운용 상태와 관계없이 std::invalid_argument를 던진다.
    // 생성자에 주입한 요청 빔 설정(빔 폭)은 BeamRequestFactory가 요청을 만들 때 반영된다.
    // 올바른 요청은 준비 상태(ON이고 자세정보를 받음)일 때만 받아들이고, 그 외에는 무시한다.
    void requestBeam(RequestBeamCommand command);

    // 송신할 빔이 없으면 std::nullopt를 반환한다. 틱마다 한 번 호출한다.
    std::optional<TransmitBeamCommand> nextBeamCommand(std::chrono::milliseconds currentTime);

private:
    std::unique_ptr<Scheduler> _scheduler;
    // RequestBeamCommand를 BeamRequest로 바꾼다. 요청 빔 설정은 생성 시 한 번 주입된다.
    std::unique_ptr<BeamRequestFactory> _requestFactory;
};

} // namespace beam
