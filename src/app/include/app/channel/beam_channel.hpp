#pragma once

#include <app/queue/message_queue.hpp>
#include <beam/dto/beam_request.hpp>
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
    beam::BeamRequest,
    NextBeamCommandEvent>;

struct ReceivedBeamEvent
{
    BeamEvent message;
};

class BeamChannel final
{
public:
    bool write(BeamEvent message);

    std::optional<ReceivedBeamEvent> read();

    void close();

private:
    MessageQueue<ReceivedBeamEvent> _queue;
};
} // namespace app