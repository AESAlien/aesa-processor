#pragma once

#include <app/queue/message_queue.hpp>
#include <beam/dto/transmit_beam_command.hpp>

#include <optional>

namespace app
{
class SimulatorChannel final
{
public:
    void write(beam::TransmitBeamCommand message);

    std::optional<beam::TransmitBeamCommand> read();

    void close();

private:
    MessageQueue<beam::TransmitBeamCommand> _queue;
};
} // namespace app
