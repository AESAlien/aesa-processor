#include <pose/controller/pose_controller.hpp>

#include <utility>

namespace pose
{
PoseController::PoseController(PoseConfig config)
    : _attitudeFactory(config.attitude),
      _antEnuTransformConfig(std::move(config.transform))
{
}

SetRadarPoseResult PoseController::setRadarPose(SetRadarPoseCommand command) const
{
    const auto attitude = _attitudeFactory.create(command.roll, command.pitch, command.yaw);
    return SetRadarPoseResult(
        SetAntEnuTransformCommand(AntEnuTransform(attitude, _antEnuTransformConfig)),
        std::move(command)
    );
}
}
