#pragma once

#include "dto/tick_event.hpp"
#include <beam/dto/set_operation_state_command.hpp>
#include <pose/dto/set_ant_enu_transform_command.hpp>
#include <beam/dto/request_beam_command.hpp>
#include <variant>

namespace app
{
using BeamInput = std::variant<
    beam::SetOperationStateCommand,
    pose::SetAntEnuTransformCommand,
    beam::RequestBeamCommand,
    TickEvent
>;
} // namespace app
