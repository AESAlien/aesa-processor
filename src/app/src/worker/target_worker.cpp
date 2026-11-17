#include "worker/target_worker.hpp"

#include <utility>
#include <variant>

namespace app
{
TargetWorker::TargetWorker(
    target::TargetController& controller,
    MessageQueue<TargetInput>& targetQueue,
    MessageQueue<BeamInput>& beamQueue,
    MessageQueue<ConsoleInput>& consoleQueue,
    std::shared_ptr<spdlog::logger> logger,
    std::shared_ptr<spdlog::logger> operationLogger
) : Worker(std::move(logger)),
    _controller(controller),
    _targetQueue(targetQueue),
    _beamQueue(beamQueue),
    _consoleQueue(consoleQueue),
    _operationLogger(std::move(operationLogger))
{
}

TargetInput TargetWorker::readInput()
{
    return _targetQueue.read();
}

std::optional<TargetOutput> TargetWorker::process(TargetInput input)
{
    if (auto* event = std::get_if<target::DetectionEvent>(&input))
    {
        // OFF에서는 새 트랙을 만들지 않도록 탐지를 버린다.
        if (!_operationState.isOn())
        {
            return std::nullopt;
        }
        return _controller.handleDetection(*event);
    }
    else if (const auto* command = std::get_if<beam::SetOperationStateCommand>(&input))
    {
        applyOperationState(command->powerState);
    }
    else if (auto* command = std::get_if<pose::SetEnuGeoTransformCommand>(&input))
    {
        _controller.setEnuGeoTransform(std::move(*command));
    }
    else if (const auto* tick = std::get_if<TickEvent>(&input))
    {
        // 응답 없는 트랙을 먼저 정리해 삭제된 트랙이 이번 틱의 보고에 포함되지 않게 한다.
        // OFF에서는 트랙이 없으므로 빈 목록을 보고해 콘솔 표시를 비운다.
        _controller.removeExpiredTracks(tick->currentTime);
        return _controller.getTrackEvents();
    }
    return std::nullopt;
}

void TargetWorker::writeOutput(TargetOutput output)
{
    if (auto* command = std::get_if<beam::RequestBeamCommand>(&output))
    {
        _beamQueue.write(std::move(*command));
    }
    else if (auto* event = std::get_if<std::vector<target::TrackEvent>>(&output))
    {
        _consoleQueue.write(std::move(*event));
    }
}

void TargetWorker::applyOperationState(beam::OperationState state)
{
    // 같은 상태의 반복 수신은 무시한다.
    if (!_operationState.apply(state))
    {
        return;
    }

    if (state == beam::OperationState::ON)
    {
        if (_operationLogger)
        {
            _operationLogger->info("Target applied operation state: ON");
        }
        return;
    }

    const auto clearedCount = _controller.clearTracks();
    if (_operationLogger)
    {
        _operationLogger->info("Target applied operation state: OFF, cleared tracks={}", clearedCount);
    }
}
} // namespace app
