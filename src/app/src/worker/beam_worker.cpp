#include "worker/beam_worker.hpp"

#include <utility>
#include <variant>

namespace app
{
BeamWorker::BeamWorker(
    beam::BeamController& controller,
    MessageQueue<BeamInput>& beamQueue,
    MessageQueue<SimulatorInput>& simulatorQueue,
    std::shared_ptr<spdlog::logger> logger,
    std::shared_ptr<spdlog::logger> operationLogger
) : Worker(std::move(logger)),
    _controller(controller),
    _beamQueue(beamQueue),
    _simulatorQueue(simulatorQueue),
    _operationLogger(std::move(operationLogger))
{
}

BeamInput BeamWorker::readInput()
{
    return _beamQueue.read();
}

std::optional<BeamOutput> BeamWorker::process(BeamInput input)
{
    if (auto* command = std::get_if<beam::RequestBeamCommand>(&input))
    {
        // OFF에서는 요청 빔을 받지 않는다.
        if (_operationState.isOn())
        {
            _controller.requestBeam(*command);
        }
    }
    else if (const auto* command = std::get_if<beam::SetOperationStateCommand>(&input))
    {
        applyOperationState(command->powerState);
    }
    else if (auto* command = std::get_if<pose::SetAntEnuTransformCommand>(&input))
    {
        // 자세정보는 운용 상태와 관계없이 반영해 ON 직후 최신 자세로 빔을 계산한다.
        _controller.setAntEnuTransform(*command);
    }
    else if (auto* tick = std::get_if<TickEvent>(&input))
    {
        // OFF에서는 빔을 송신하지 않는다.
        if (!_operationState.isOn())
        {
            return std::nullopt;
        }
        return _controller.nextBeamCommand(tick->currentTime);
    }
    return std::nullopt;
}

void BeamWorker::writeOutput(BeamOutput output)
{
    _simulatorQueue.write(std::move(output));
}

void BeamWorker::applyOperationState(beam::OperationState state)
{
    // 같은 상태의 반복 수신은 무시한다.
    if (!_operationState.apply(state))
    {
        return;
    }

    // 전환마다 요청 큐, 송신 횟수, 탐색 순서를 초기화해 이전 운용의 요청이 남지 않게 한다.
    _controller.reset();
    if (_operationLogger)
    {
        _operationLogger->info(
            "Beam applied operation state: {}",
            state == beam::OperationState::ON ? "ON" : "OFF"
        );
    }
}
} // namespace app
