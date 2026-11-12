#include <app/worker/simulator_send_worker.hpp>
#include <app/worker/worker_error.hpp>

#include <utility>

namespace app
{
SimulatorSendWorker::SimulatorSendWorker(
    SimulatorChannel& simulatorChannel,
    SendBeamCommand sendBeamCommand
) : _simulatorChannel(simulatorChannel),
    _sendBeamCommand(std::move(sendBeamCommand))
{
}

void SimulatorSendWorker::run()
{
    while (auto command = _simulatorChannel.read())
    {
        tryWorkerAction("CommSendSimulator", [&]
        {
            _sendBeamCommand(*command);
        });
    }
}
} // namespace app
