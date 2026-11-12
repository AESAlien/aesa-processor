#include <app/channel/simulator_channel.hpp>

#include <utility>

namespace app
{
void SimulatorChannel::write(beam::TransmitBeamCommand message)
{
    _queue.push(std::move(message));
}

std::optional<beam::TransmitBeamCommand> SimulatorChannel::read()
{
    return _queue.pop();
}

void SimulatorChannel::close()
{
    _queue.close();
}
} // namespace app
