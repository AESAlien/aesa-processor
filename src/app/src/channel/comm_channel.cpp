#include <app/channel/comm_channel.hpp>

#include <utility>
#include <variant>

namespace app
{
void CommChannel::write(CommEvent message)
{
    if (std::holds_alternative<comm::MonitorStatusRequest>(message))
    {
        _monitorQueue.push(std::move(message));
        return;
    }

    _monitorQueue.push(message);

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

std::optional<CommEvent> CommChannel::readMonitor()
{
    return _monitorQueue.pop();
}

void CommChannel::close()
{
    _simulatorQueue.close();
    _consoleQueue.close();
    _monitorQueue.close();
}
} // namespace app