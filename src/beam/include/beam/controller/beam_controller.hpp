#pragma once

#include <beam/dto/beam_command.hpp>
#include <beam/dto/beam_request.hpp>
#include <beam/dto/set_attitude_command.hpp>
#include <beam/dto/set_operation_state_command.hpp>

namespace beam
{

class BeamController
{
public:
    void setOperationState(SetOperationStateCommand command);
    void setAttitude(SetAttitudeCommand command);
    void requestBeam(BeamRequest beamRequest);
    BeamCommand nextBeamCommand();
};

} // namespace beam
