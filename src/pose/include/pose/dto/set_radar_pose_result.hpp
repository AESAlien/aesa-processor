#pragma once

#include <pose/dto/set_radar_pose_command.hpp>
#include <pose/dto/set_ant_enu_transform_command.hpp>

#include <utility>

namespace pose
{
struct SetRadarPoseResult
{
    SetRadarPoseResult(
        SetAntEnuTransformCommand antEnuTransform,
        SetRadarPoseCommand pose
    ) : antEnuTransform(std::move(antEnuTransform)),
        pose(std::move(pose))
    {
    }

    SetAntEnuTransformCommand antEnuTransform;
    SetRadarPoseCommand pose;
};
}
