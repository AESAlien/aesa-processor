#pragma once

#include <app/queue/message_queue.hpp>
#include <beam/dto/beam_command.hpp>
#include <target/dto/track_event.hpp>

#include <cstdint>
#include <optional>
#include <variant>
#include <vector>

namespace app
{
using CommEvent = std::variant<
    beam::BeamCommand,
    std::vector<target::TrackEvent>>;

struct ReceivedCommEvent
{
    CommEvent message;
    std::int64_t rxTimeNs;
};

class CommChannel final
{
public:
    bool write(CommEvent message);
    bool write(const target::TrackEvent& message);

    std::optional<ReceivedCommEvent> readSimulator();
    std::optional<ReceivedCommEvent> readConsole();

    void close();

private:
    MessageQueue<ReceivedCommEvent> _simulatorQueue;
    MessageQueue<ReceivedCommEvent> _consoleQueue;
};
} // namespace app