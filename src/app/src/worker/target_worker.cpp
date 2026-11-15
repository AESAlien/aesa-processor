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
    if (auto* event = std::get_if<target::DetectionEvent>(&input))
    {
        return _controller.handleDetection(*event);
    }
    else if (auto* command = std::get_if<pose::SetEnuGeoTransformCommand>(&input))
    {
        _controller.setEnuGeoTransform(std::move(*command));
    }
    else if (const auto* tick = std::get_if<TickEvent>(&input))
    {
        // 응답 없는 트랙을 먼저 정리해 삭제된 트랙이 이번 틱의 보고에 포함되지 않게 한다.
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
} // namespace app
