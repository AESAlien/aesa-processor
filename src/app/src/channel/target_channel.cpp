#include <app/channel/target_channel.hpp>

#include <utility>

namespace aesa_processor::app
{
void TargetChannel::write(TargetEvent message)
{
    _queue.push(std::move(message));
}

std::optional<TargetEvent> TargetChannel::read()
{
    return _queue.pop();
}

void TargetChannel::close()
{
    _queue.close();
}
} // namespace aesa_processor::app