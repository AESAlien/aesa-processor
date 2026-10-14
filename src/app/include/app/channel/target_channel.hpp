#pragma once

#include <app/queue/message_queue.hpp>
#include <target/dto/detection_event.hpp>

#include <optional>

namespace app
{
using TargetEvent = target::DetectionEvent;

class TargetChannel final
{
public:
    void write(TargetEvent message);

    std::optional<TargetEvent> read();

    void close();

private:
    MessageQueue<TargetEvent> _queue;
};
} // namespace app