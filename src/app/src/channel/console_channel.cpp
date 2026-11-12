#include <app/channel/console_channel.hpp>

#include <utility>

namespace app
{
void ConsoleChannel::write(ConsoleEvent message)
{
    _queue.push(std::move(message));
}

void ConsoleChannel::write(const target::TrackEvent& message)
{
    write(ConsoleEvent{std::vector<target::TrackEvent>{message}});
}

std::optional<ConsoleEvent> ConsoleChannel::read()
{
    return _queue.pop();
}

void ConsoleChannel::close()
{
    _queue.close();
}
} // namespace app
