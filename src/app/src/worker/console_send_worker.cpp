#include "worker/console_send_worker.hpp"

#include <utility>
#include <variant>

namespace app
{
ConsoleSendWorker::ConsoleSendWorker(
    comm::CommController& controller,
    MessageQueue<ConsoleInput>& consoleQueue,
    std::shared_ptr<spdlog::logger> logger
) : Worker(std::move(logger)),
    _controller(controller),
    _consoleQueue(consoleQueue)
{
}

ConsoleInput ConsoleSendWorker::readInput()
{
    return _consoleQueue.read();
}

std::optional<ConsoleSendOutput> ConsoleSendWorker::process(ConsoleInput input)
{
    if (const auto* command = std::get_if<pose::SetRadarPoseCommand>(&input))
    {
        _controller.sendRadarPose(*command);
    }
    else if (const auto* event = std::get_if<std::vector<target::TrackEvent>>(&input))
    {
        _controller.sendTrackEvents(*event);
    }
    return std::nullopt;
}
} // namespace app
