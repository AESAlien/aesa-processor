#include "worker/simulator_send_worker.hpp"

#include <utility>

namespace app
{
SimulatorSendWorker::SimulatorSendWorker(
    comm::CommController& controller,
    MessageQueue<SimulatorInput>& simulatorQueue,
    std::shared_ptr<spdlog::logger> logger
) : Worker(std::move(logger)),
    _controller(controller),
    _simulatorQueue(simulatorQueue)
{
}

SimulatorInput SimulatorSendWorker::readInput()
{
    return _simulatorQueue.read();
}

std::monostate SimulatorSendWorker::process(SimulatorInput input)
{
    _controller.sendBeamCommand(input);
    return {};
}
} // namespace app
