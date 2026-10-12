#include <app/channel/beam_channel.hpp>

#include <chrono>
#include <utility>

namespace app
{
bool BeamChannel::write(BeamEvent message)
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    const auto rxTimeNs =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();

    return _queue.push(ReceivedBeamEvent{std::move(message), rxTimeNs});
}

std::optional<ReceivedBeamEvent> BeamChannel::read()
{
    return _queue.pop();
}

void BeamChannel::close()
{
    _queue.close();
}
} // namespace app