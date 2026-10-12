#include <app/channel/comm_channel.hpp>

#include <utility>
#include <variant>

namespace app
{
bool CommChannel::write(CommEvent message)
{
    const bool toSimulator =
        std::holds_alternative<beam::BeamCommand>(message);

    ReceivedCommEvent event{
        std::move(message)
    };

    if (toSimulator)
    {
        return _simulatorQueue.push(std::move(event));
    }

    return _consoleQueue.push(std::move(event));
}

bool CommChannel::write(const target::TrackEvent& message)
{
    return write(
        CommEvent{std::vector<target::TrackEvent>{message}});
}

std::optional<ReceivedCommEvent> CommChannel::readSimulator()
{
    return _simulatorQueue.pop();
}

std::optional<ReceivedCommEvent> CommChannel::readConsole()
{
    return _consoleQueue.pop();
}

void CommChannel::close()
{
    _simulatorQueue.close();
    _consoleQueue.close();
}
} // namespace app