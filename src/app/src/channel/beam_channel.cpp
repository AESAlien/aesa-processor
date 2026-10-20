#include <app/channel/beam_channel.hpp>

#include <utility>

namespace aesa_processor::app
{
void BeamChannel::write(BeamEvent message)
{
    _queue.push(std::move(message));
}

std::optional<BeamEvent> BeamChannel::read()
{
    return _queue.pop();
}

void BeamChannel::close()
{
    _queue.close();
}
} // namespace aesa_processor::app