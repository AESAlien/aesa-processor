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
    std::shared_ptr<spdlog::logger> logger
) : Worker(std::move(logger)),
    _controller(controller),
    _targetQueue(targetQueue),
    _beamQueue(beamQueue),
    _consoleQueue(consoleQueue)
{
}

TargetInput TargetWorker::readInput()
{
    return _targetQueue.read();
}

std::optional<TargetOutput> TargetWorker::process(TargetInput input)
{
    if (auto* detection = std::get_if<target::DetectionEvent>(&input))
    {
        auto request = _controller.handleDetection(*detection);
        if (request)
        {
            return TargetOutput{std::move(*request)};
        }
    }
    else if (const auto* tick = std::get_if<TickEvent>(&input))
    {
        // 응답 없는 트랙을 먼저 정리해 삭제된 트랙이 이번 틱의 보고에 포함되지 않게 한다.
        _controller.removeExpiredTracks(tick->currentTime);
        return TargetOutput{_controller.nextTrackEvents()};
    }
    return std::nullopt;
}

void TargetWorker::writeOutput(std::optional<TargetOutput> output)
{
    if (!output)
    {
        return;
    }
    if (auto* request = std::get_if<beam::RequestBeamCommand>(&*output))
    {
        _beamQueue.write(std::move(*request));
    }
    else if (auto* events = std::get_if<std::vector<target::TrackEvent>>(&*output))
    {
        _consoleQueue.write(std::move(*events));
    }
}
} // namespace app
