#pragma once

#include <pose/domain/factory/radar_attitude_factory.hpp>

#include <pose/config/pose_config.hpp>
#include <pose/dto/set_radar_pose_result.hpp>

namespace pose
{
class PoseController
{
public:
    explicit PoseController(PoseConfig config);

    SetRadarPoseResult setRadarPose(SetRadarPoseCommand command) const;

private:
    RadarAttitudeFactory _attitudeFactory;
    AntEnuTransformConfig _antEnuTransformConfig;
};
}
