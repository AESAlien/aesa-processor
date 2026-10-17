#pragma once

#include <beam/config/beam_config.hpp>
#include <beam/dto/beam_command.hpp>
#include <beam/dto/beam_request.hpp>
#include <beam/dto/set_attitude_command.hpp>
#include <beam/dto/set_operation_state_command.hpp>

#include <chrono>
#include <memory>
#include <optional>

namespace beam
{

class Scheduler;

// 빔 모듈의 외부 인터페이스. 내부 스케줄러에 호출을 전달한다.
// 스레드 안전하지 않으므로 모든 메서드를 같은 스레드에서 호출해야 한다.
class BeamController
{
public:
    // 기본 설정(BeamConfig{})으로 만든다.
    BeamController();
    // 주어진 설정으로 만든다. 설정은 BeamConfig 생성 시 이미 검증되어 있다.
    explicit BeamController(const BeamConfig& config);
    ~BeamController();

    BeamController(BeamController&&) noexcept;
    BeamController& operator=(BeamController&&) noexcept;

    void setOperationState(SetOperationStateCommand command);

    // 자세정보가 잘못되면 운용 상태와 관계없이 예외(AttitudeError)를 던진다.
    void setAttitude(SetAttitudeCommand command);

    void requestBeam(BeamRequest beamRequest);

    // 송신할 빔이 없으면 std::nullopt를 반환한다. 틱마다 한 번 호출한다.
    std::optional<BeamCommand> nextBeamCommand(std::chrono::milliseconds currentTime);

private:
    std::unique_ptr<Scheduler> _scheduler;
};

} // namespace beam
