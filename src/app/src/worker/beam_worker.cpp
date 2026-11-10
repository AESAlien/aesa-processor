#include <app/worker/beam_worker.hpp>
#include <app/worker/worker_error.hpp>

#include <magic_enum/magic_enum.hpp>
#include <spdlog/spdlog.h>

#include <iostream>
#include <utility>
#include <variant>

namespace app
{
namespace
{
// main에서 등록한 로거 이름과 같아야 한다.
constexpr char WORKER_NAME[] = "BeamWorker";

// OperationState는 bool 기반 enum이므로 magic_enum 대신 직접 변환한다.
const char* operationStateName(beam::OperationState state)
{
    return state == beam::OperationState::ON ? "ON" : "OFF";
}
} // namespace

BeamWorker::BeamWorker(
    BeamChannel& beamChannel,
    CommChannel& commChannel,
    beam::BeamController& controller)
    : _beamChannel(beamChannel),
      _commChannel(commChannel),
      _controller(controller),
      _logger(spdlog::get(WORKER_NAME))
{
}

void BeamWorker::run()
{
    while (auto event = _beamChannel.read())
    {
        // 예외는 tryWorkerAction이 로그 파일에 error로 기록한다.
        tryWorkerAction(WORKER_NAME, [&]
        {
            auto& message = *event;

            if (auto* request =
                    std::get_if<beam::RequestBeamCommand>(&message))
            {
                const auto result = _controller.requestBeam(*request);

                // 수락은 표적 수에 비례해 반복되므로 출력하지 않고, 거부만 기록한다.
                if (result.status != beam::RequestBeamStatus::ACCEPTED && _logger)
                {
                    _logger->warn(
                        "Beam request {}: requestId={}",
                        magic_enum::enum_name(result.status),
                        result.requestId);
                }
            }
            else if (auto* state =
                         std::get_if<beam::SetOperationStateCommand>(&message))
            {
                const auto status = _controller.setOperationState(*state);

                // 같은 상태의 중복 수신(UNCHANGED)은 출력하지 않는다.
                if (status == beam::SetOperationStateStatus::CHANGED)
                {
                    std::cout
                        << "[BEAM] Operation state: "
                        << operationStateName(state->powerState)
                        << '\n';
                }
            }
            else if (auto* attitude =
                         std::get_if<beam::SetAttitudeCommand>(&message))
            {
                const auto status = _controller.setAttitude(*attitude);

                if (status == beam::SetAttitudeStatus::APPLIED)
                {
                    std::cout
                        << "[BEAM] Attitude applied: roll="
                        << attitude->roll.deg()
                        << ", pitch="
                        << attitude->pitch.deg()
                        << ", yaw="
                        << attitude->yaw.deg()
                        << '\n';
                }
                else if (_logger)
                {
                    _logger->warn(
                        "Attitude {}: roll={}, pitch={}, yaw={}",
                        magic_enum::enum_name(status),
                        attitude->roll.deg(),
                        attitude->pitch.deg(),
                        attitude->yaw.deg());
                }
            }
            else if (auto* tick =
                         std::get_if<NextBeamCommandEvent>(&message))
            {
                auto command =
                    _controller.nextBeamCommand(tick->currentTime);

                if (command)
                {
                    _commChannel.write(std::move(*command));
                }
            }
        });
    }
}

} // namespace app