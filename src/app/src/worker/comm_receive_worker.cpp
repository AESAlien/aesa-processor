#include "worker/comm_receive_worker.hpp"

#include <utility>
#include <variant>

namespace app
{
CommReceiveWorker::CommReceiveWorker(
    comm::CommController& controller,
    MessageQueue<OperationInput>& operationQueue,
    MessageQueue<TargetInput>& targetQueue,
    MessageQueue<PoseInput>& poseQueue,
    std::shared_ptr<spdlog::logger> logger
) : Worker(std::move(logger)),
    _controller(controller),
    _operationQueue(operationQueue),
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
        // 틱 워커가 시간 기준을 반영한 뒤 틱과 같은 순서로 빔·타겟에 전달한다.
        _operationQueue.write(std::move(*command));
    }
    else if (auto* event = std::get_if<target::DetectionEvent>(&output))
    {
        _targetQueue.write(std::move(*event));
    }
}
} // namespace app
