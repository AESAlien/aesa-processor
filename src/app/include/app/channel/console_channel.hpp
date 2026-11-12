#pragma once

#include <app/queue/message_queue.hpp>
#include <beam/dto/set_attitude_command.hpp>
#include <target/dto/track_event.hpp>

#include <variant>
#include <vector>

#include <optional>

namespace app
{
using ConsoleEvent = std::variant<beam::SetAttitudeCommand, std::vector<target::TrackEvent>>;

class ConsoleChannel final
{
public:
    void write(ConsoleEvent message);

    void write(const target::TrackEvent& message);

    std::optional<ConsoleEvent> read();

    void close();

private:
    MessageQueue<ConsoleEvent> _queue;
};
} // namespace app
