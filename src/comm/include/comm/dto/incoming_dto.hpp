#pragma once

#include <beam/dto/set_attitude_command.hpp>
#include <beam/dto/set_operation_state_command.hpp>
#include <target/dto/detection_event.hpp>

#include <variant>

namespace aesa_processor::comm
{

using IncomingDto = std::variant<
    aesa_processor::beam::SetOperationStateCommand,
    aesa_processor::beam::SetAttitudeCommand,
    aesa_processor::target::DetectionEvent
>;

} // namespace aesa_processor::comm
