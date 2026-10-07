#pragma once

#include <beam/dto/beam_command.hpp>
#include <beam/dto/set_attitude_command.hpp>
#include <beam/dto/set_operation_state_command.hpp>
#include <comm/network/network_types.hpp>
#include <target/dto/detection_event.hpp>
#include <target/dto/track_event.hpp>

#include <chrono>
#include <memory>
#include <variant>
#include <vector>

namespace comm
{

struct CommConfiguration
{
    NetworkEndpoint server;
    NetworkEndpoint operatorConsole;
    NetworkEndpoint scenarioSimulator;
};

using IncomingDto = std::variant<
    beam::SetOperationStateCommand,
    beam::SetAttitudeCommand,
    target::DetectionEvent
>;

enum class ReceiveStatus
{
    EVENTS,
    TIMEOUT,
    NOT_STARTED,
    INVALID_ARGUMENT,
    NETWORK_ERROR,
    PROTOCOL_ERROR
};

struct ReceiveResult
{
    ReceiveStatus status = ReceiveStatus::TIMEOUT;
    std::vector<IncomingDto> dtos;
    NetworkOperation failedOperation = NetworkOperation::NONE;
    int errorCode = 0;
};

// Facade for network connection, framing, protocol conversion, and transport.
// Call every method from the same communication worker thread.
class CommController
{
public:
    explicit CommController(CommConfiguration configuration);
    ~CommController();

    CommController(const CommController&) = delete;
    CommController& operator=(const CommController&) = delete;
    CommController(CommController&&) = delete;
    CommController& operator=(CommController&&) = delete;

    OperationResult start();

    ReceiveResult receive(std::chrono::milliseconds timeout);

    SendResult sendBeamCommand(const beam::BeamCommand& command);

    SendResult sendTrackEvents(
        const std::vector<target::TrackEvent>& trackEvents
    );

    OperationResult stop();

private:
    class Impl;
    std::unique_ptr<Impl> _impl;
};

} // namespace comm
