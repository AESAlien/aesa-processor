#pragma once

#include <pose/dto/set_radar_pose_command.hpp>
#include <target/dto/track_event.hpp>
#include <variant>
#include <vector>

namespace app
{
using ConsoleInput = std::variant<
    pose::SetRadarPoseCommand,
    std::vector<target::TrackEvent>
>;
} // namespace app
