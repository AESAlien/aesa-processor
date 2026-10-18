#pragma once

#include <app/queue/message_queue.hpp>
#include <beam/dto/request_beam_command.hpp>
#include <beam/dto/set_attitude_command.hpp>
#include <beam/dto/set_operation_state_command.hpp>

#include <chrono>
#include <optional>
#include <variant>

namespace app
{
struct NextBeamCommandEvent
{
    std::chrono::milliseconds currentTime{0};
};

using BeamEvent = std::variant<
    beam::SetOperationStateCommand,
    beam::SetAttitudeCommand,
    beam::RequestBeamCommand,
    NextBeamCommandEvent>;

class BeamChannel final
{
public:
    void write(BeamEvent message);

    std::optional<BeamEvent> read();

    void close();

private:
    MessageQueue<BeamEvent> _queue;
};
} // namespace app