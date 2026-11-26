#include "worker/target_worker.hpp"

#include <utility>
#include <variant>

namespace app
{
namespace
{
// 탐지의 송신 시각이 마지막 틱보다 이만큼 넘게 앞서면, ON 전환 전에 보낸 빔의 늦은 결과로 본다.
// 정상 탐지는 그 빔의 틱이 먼저 들어오므로 앞서지 않으며, 여유는 틱 지연을 감안한 값이다.
constexpr std::chrono::milliseconds MAX_DETECTION_LEAD{1000};
} // namespace

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
        // ON에서 시각을 0부터 다시 세므로, OFF 전에 보낸 빔의 늦은 결과는 미래 시각으로 보인다.
        if (event->transmitTime > _currentTime + MAX_DETECTION_LEAD)
        {
            if (_operationLogger)
            {
                _operationLogger->info(
                    "Target dropped detection sent before operation ON: transmitTime={}ms, currentTime={}ms",
                    event->transmitTime.count(),
                    _currentTime.count()
                );
            }
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
        _currentTime = tick->currentTime;
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
        // 틱 워커는 ON 명령 바로 뒤에 시각 0의 틱을 보낸다.
        _currentTime = std::chrono::milliseconds{0};
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
