#include "worker/comm_receive_worker.hpp"

#include <utility>
#include <variant>

namespace app
{
CommReceiveWorker::CommReceiveWorker(
    comm::CommController& controller,
    MessageQueue<BeamInput>& beamQueue,
    MessageQueue<TargetInput>& targetQueue,
    MessageQueue<ConsoleInput>& consoleQueue,
    std::shared_ptr<spdlog::logger> logger
) : Worker(std::move(logger)),
    _controller(controller),
    _beamQueue(beamQueue),
    _targetQueue(targetQueue),
    _consoleQueue(consoleQueue)
{
}

std::optional<comm::IncomingDto> CommReceiveWorker::readInput()
{
    return _controller.receiveData();
}

std::optional<comm::IncomingDto> CommReceiveWorker::process(
    std::optional<comm::IncomingDto> input
) {
    return input;
}

void CommReceiveWorker::writeOutput(std::optional<comm::IncomingDto> output)
{
    if (!output)
    {
        return;
    }
    if (auto* attitude = std::get_if<beam::SetAttitudeCommand>(&*output))
    {
        _consoleQueue.write(*attitude);
        _beamQueue.write(*attitude);
    }
    else if (auto* state = std::get_if<beam::SetOperationStateCommand>(&*output))
    {
        _beamQueue.write(std::move(*state));
    }
    else if (auto* detection = std::get_if<target::DetectionEvent>(&*output))
    {
        _targetQueue.write(std::move(*detection));
    }
}
} // namespace app
