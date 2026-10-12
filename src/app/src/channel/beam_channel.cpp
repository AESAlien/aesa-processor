#include <app/channel/beam_channel.hpp>

#include <utility>

namespace app
{
bool BeamChannel::write(BeamEvent message)
{
    return _queue.push(
        ReceivedBeamEvent{std::move(message)});
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