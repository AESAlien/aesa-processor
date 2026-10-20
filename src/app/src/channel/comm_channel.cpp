#include <app/channel/comm_channel.hpp>

#include <utility>
#include <variant>

namespace app
{
void CommChannel::write(CommEvent message)
{
    if (std::holds_alternative<beam::TransmitBeamCommand>(message))
    {
        _simulatorQueue.push(std::move(message));
    }
    else
    {
        _consoleQueue.push(std::move(message));
    }
}

void CommChannel::write(const target::TrackEvent& message)
{
    write(
        CommEvent{std::vector<target::TrackEvent>{message}});
}

std::optional<CommEvent> CommChannel::readSimulator()
{
    return _simulatorQueue.pop();
}

std::optional<CommEvent> CommChannel::readConsole()
{
    return _consoleQueue.pop();
}

void CommChannel::close()
{
    _simulatorQueue.close();
    _consoleQueue.close();
}
} // namespace app