#include <app/channel/comm_channel.hpp>

#include <chrono>
#include <utility>
#include <variant>

namespace app
{
bool CommChannel::write(CommEvent message)
{
    const auto now =
        std::chrono::steady_clock::now().time_since_epoch();

    const auto timestamp =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();

    const bool toSimulator =
        std::holds_alternative<beam::BeamCommand>(message);

    ReceivedCommEvent event{
        std::move(message),
        timestamp
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