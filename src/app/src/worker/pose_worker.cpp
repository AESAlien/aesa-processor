#include "worker/pose_worker.hpp"

#include <utility>

namespace app
{
PoseWorker::PoseWorker(
    const pose::PoseController& controller,
    MessageQueue<PoseInput>& poseQueue,
    MessageQueue<BeamInput>& beamQueue,
    MessageQueue<ConsoleInput>& consoleQueue,
    std::shared_ptr<spdlog::logger> logger
) : Worker(std::move(logger)),
    _controller(controller),
    _poseQueue(poseQueue),
    _beamQueue(beamQueue),
    _consoleQueue(consoleQueue)
{
}

PoseInput PoseWorker::readInput()
{
    return _poseQueue.read();
}

std::optional<PoseOutput> PoseWorker::process(PoseInput input)
{
    return _controller.setRadarPose(std::move(input));
}

void PoseWorker::writeOutput(pose::SetRadarPoseResult output)
{
    _beamQueue.write(std::move(output.antEnuTransform));
    _consoleQueue.write(std::move(output.pose));
}
}
