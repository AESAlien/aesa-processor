#include <app/channel/target_channel.hpp>

#include <chrono>
#include <utility>

namespace app
{
bool TargetChannel::write(TargetEvent message)
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    const auto rxTimeNs =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
    return _queue.push(ReceivedTargetEvent{std::move(message), rxTimeNs});
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
