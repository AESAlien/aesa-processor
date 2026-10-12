#pragma once

#include <app/queue/message_queue.hpp>
#include <beam/dto/set_operation_state_command.hpp>
#include <beam/dto/set_attitude_command.hpp>
#include <beam/dto/beam_request.hpp>

#include <cstdint>
#include <optional>
#include <variant>

namespace app
{
struct NextBeamCommandEvent
{
};

using BeamEvent = std::variant<
    beam::SetOperationStateCommand,
    beam::SetAttitudeCommand,
    beam::BeamRequest,
    NextBeamCommandEvent>;

struct ReceivedBeamEvent
{
    BeamEvent message;
    std::int64_t rxTimeNs;
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