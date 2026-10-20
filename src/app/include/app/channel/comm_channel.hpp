#pragma once

#include <app/queue/message_queue.hpp>
#include <beam/dto/transmit_beam_command.hpp>
#include <target/dto/track_event.hpp>

#include <optional>
#include <variant>
#include <vector>

namespace app
{
using CommEvent = std::variant<
    beam::TransmitBeamCommand,
    std::vector<target::TrackEvent>>;

class CommChannel final
{
public:
    void write(CommEvent message);

    void write(const target::TrackEvent& message);

    std::optional<CommEvent> readSimulator();

    std::optional<CommEvent> readConsole();

    void close();

private:
    MessageQueue<CommEvent> _simulatorQueue;
    MessageQueue<CommEvent> _consoleQueue;
};
} // namespace app