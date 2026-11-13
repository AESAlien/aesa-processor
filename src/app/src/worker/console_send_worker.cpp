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

std::monostate ConsoleSendWorker::process(ConsoleInput input)
{
    if (const auto* attitude = std::get_if<beam::SetAttitudeCommand>(&input))
    {
        _controller.sendRadarAttitude(*attitude);
    }
    else if (const auto* events = std::get_if<std::vector<target::TrackEvent>>(&input))
    {
        _controller.sendTrackEvents(*events);
    }
    return {};
}
} // namespace app
