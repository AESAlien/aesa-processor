#include "worker/comm_receive_worker.hpp"

#include <utility>
#include <variant>

namespace app
{
CommReceiveWorker::CommReceiveWorker(
    comm::CommController& controller,
    MessageQueue<BeamInput>& beamQueue,
    MessageQueue<TargetInput>& targetQueue,
    MessageQueue<PoseInput>& poseQueue,
    std::shared_ptr<spdlog::logger> logger
) : Worker(std::move(logger)),
    _controller(controller),
    _beamQueue(beamQueue),
    _targetQueue(targetQueue),
    _poseQueue(poseQueue)
{
}

std::monostate CommReceiveWorker::readInput()
{
    return {};
}

std::optional<CommReceiveOutput> CommReceiveWorker::process(std::monostate)
{
    return _controller.receiveData();
}

void CommReceiveWorker::writeOutput(CommReceiveOutput output)
{
    if (auto* command = std::get_if<pose::SetRadarPoseCommand>(&output))
    {
        _poseQueue.write(std::move(*command));
    }
    else if (auto* command = std::get_if<beam::SetOperationStateCommand>(&output))
    {
        // 빔과 타겟이 각자 자기 큐 순서대로 운용 상태를 반영하도록 양쪽에 보낸다.
        _beamQueue.write(*command);
        _targetQueue.write(std::move(*command));
    }
    else if (auto* event = std::get_if<target::DetectionEvent>(&output))
    {
        _targetQueue.write(std::move(*event));
    }
}
} // namespace app
