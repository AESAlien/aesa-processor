#pragma once

#include <beam/dto/beam_command.hpp>
#include <beam/dto/set_attitude_command.hpp>
#include <beam/dto/set_operation_state_command.hpp>
#include <target/dto/detection_event.hpp>
#include <target/dto/track_event.hpp>

#include <variant>
#include <vector>

namespace comm
{

class CommController
{
public:
    void openServer();

    std::variant<
        beam::SetAttitudeCommand,
        beam::SetOperationStateCommand,
        target::DetectionEvent
    > receiveData();

    void sendBeamCommand(const beam::BeamCommand& command);
    void sendTrackEvents(const std::vector<target::TrackEvent>& trackEvents);
};

} // namespace comm
