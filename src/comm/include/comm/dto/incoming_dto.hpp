#pragma once

#include <beam/dto/set_attitude_command.hpp>
#include <beam/dto/set_operation_state_command.hpp>
#include <target/dto/detection_event.hpp>

#include <variant>

namespace comm
{

using IncomingDto = std::variant<
    beam::SetOperationStateCommand,
    beam::SetAttitudeCommand,
    target::DetectionEvent
>;

} // namespace comm
