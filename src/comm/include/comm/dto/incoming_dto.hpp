#pragma once

#include <pose/dto/set_radar_pose_command.hpp>
#include <beam/dto/set_operation_state_command.hpp>
#include <target/dto/detection_event.hpp>

#include <variant>

namespace comm
{

using IncomingDto = std::variant<
    beam::SetOperationStateCommand,
    pose::SetRadarPoseCommand,
    target::DetectionEvent
>;

} // namespace comm
