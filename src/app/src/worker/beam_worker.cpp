#include "worker/beam_worker.hpp"

#include <utility>
#include <variant>

namespace app
{
BeamWorker::BeamWorker(
    beam::BeamController& controller,
    MessageQueue<BeamInput>& beamQueue,
    MessageQueue<SimulatorInput>& simulatorQueue,
    std::shared_ptr<spdlog::logger> logger
) : Worker(std::move(logger)),
    _controller(controller),
    _beamQueue(beamQueue),
    _simulatorQueue(simulatorQueue)
{
}

BeamInput BeamWorker::readInput()
{
    return _beamQueue.read();
}

std::optional<SimulatorInput> BeamWorker::process(BeamInput input)
{
    if (auto* command = std::get_if<beam::RequestBeamCommand>(&input))
    {
        _controller.requestBeam(*command);
    }
    else if (auto* command = std::get_if<beam::SetOperationStateCommand>(&input))
    {
        _controller.setOperationState(*command);
    }
    else if (auto* command = std::get_if<beam::SetAttitudeCommand>(&input))
    {
        _controller.setAttitude(*command);
    }
    else if (auto* event = std::get_if<TickEvent>(&input))
    {
        return _controller.nextBeamCommand(event->currentTime);
    }
    return std::nullopt;
}

void BeamWorker::writeOutput(std::optional<SimulatorInput> output)
{
    if (output)
    {
        _simulatorQueue.write(std::move(*output));
    }
}
} // namespace app
