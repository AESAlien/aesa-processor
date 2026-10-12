#include <app/channel/target_channel.hpp>

#include <utility>

namespace app
{
bool TargetChannel::write(TargetEvent message)
{
    return _queue.push(
        ReceivedTargetEvent{std::move(message)});
}

std::optional<ReceivedTargetEvent> TargetChannel::read()
{
    return _queue.pop();
}

void TargetChannel::close()
{
    _queue.close();
}
} // namespace app