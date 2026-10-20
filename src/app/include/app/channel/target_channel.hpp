#pragma once

#include <app/queue/message_queue.hpp>
#include <target/dto/detection_event.hpp>

#include <optional>

namespace aesa_processor::app
{
using TargetEvent = aesa_processor::target::DetectionEvent;

class TargetChannel final
{
public:
    void write(TargetEvent message);

    std::optional<TargetEvent> read();

    void close();

private:
    MessageQueue<TargetEvent> _queue;
};
} // namespace aesa_processor::app