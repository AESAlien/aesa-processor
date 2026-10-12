#pragma once

#include <app/queue/message_queue.hpp>
#include <target/dto/detection_event.hpp>

#include <cstdint>
#include <optional>
#include <variant>

namespace app
{
using TargetEvent = std::variant<
    target::DetectionEvent>;

struct ReceivedTargetEvent
{
    TargetEvent message;
    std::int64_t rxTimeNs;
};

class TargetChannel final
{
public:
    bool write(TargetEvent message);
    std::optional<ReceivedTargetEvent> read();
    void close();

private:
    MessageQueue<ReceivedTargetEvent> _queue;
};
} // namespace app
