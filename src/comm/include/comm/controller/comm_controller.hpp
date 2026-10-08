#pragma once

#include <beam/dto/beam_command.hpp>
#include <beam/dto/set_attitude_command.hpp>
#include <beam/dto/set_operation_state_command.hpp>
#include <target/dto/detection_event.hpp>
#include <target/dto/track_event.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace comm
{

struct CommEndpoint
{
    std::string address;
    std::uint16_t port = 0;
};

struct CommConfiguration
{
    CommEndpoint server;
    CommEndpoint operatorConsole;
    CommEndpoint scenarioSimulator;
};

using IncomingDto = std::variant<
    beam::SetOperationStateCommand,
    beam::SetAttitudeCommand,
    target::DetectionEvent
>;

class CommController
{
public:
    explicit CommController(CommConfiguration configuration);
    ~CommController();

    CommController(const CommController&) = delete;
    CommController& operator=(const CommController&) = delete;
    CommController(CommController&&) = delete;
    CommController& operator=(CommController&&) = delete;

    void openServer();

    std::optional<IncomingDto> receiveData();

    void sendBeamCommand(
        const beam::BeamCommand& command
    );

    void sendTrackEvents(
        const std::vector<target::TrackEvent>& trackEvents
    );

    // void requestStop() noexcept;
    // void closeServer() noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> _impl;
};

} // namespace comm
