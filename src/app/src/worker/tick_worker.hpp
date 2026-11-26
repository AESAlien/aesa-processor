#pragma once

#include "worker/worker.hpp"

#include "dto/beam_input.hpp"
#include "dto/operation_input.hpp"
#include "dto/target_input.hpp"
#include "dto/tick_event.hpp"
#include "operation/operation_state_gate.hpp"
#include "queue/message_queue.hpp"
#include <chrono>
#include <optional>
#include <variant>
#include <vector>

namespace app
{
struct TickOutput
{
    // 이번 틱보다 먼저 전달할 운용 명령. 받은 순서대로 담는다.
    std::vector<OperationInput> operationCommands;
    TickEvent tick;
};

// 10ms마다 틱을 만들고, 운용 명령을 틱과 같은 순서로 빔·타겟 큐에 전달한다.
// 운용 ON으로 전환되면 시간 기준을 다시 잡아 ON 직후 틱의 시각을 0으로 만든다.
// 명령과 틱을 이 워커만 쓰므로, ON 명령 뒤에 오는 틱은 두 큐 모두에서 새 기준의 시각을 갖는다.
class TickWorker final : public Worker<std::monostate, TickOutput>
{
public:
    TickWorker(
        MessageQueue<OperationInput>& operationQueue,
        MessageQueue<BeamInput>& beamQueue,
        MessageQueue<TargetInput>& targetQueue,
        std::shared_ptr<spdlog::logger> logger,
        std::shared_ptr<spdlog::logger> operationLogger
    );

private:
    std::monostate readInput() override;
    std::optional<TickOutput> process(std::monostate input) override;
    void writeOutput(TickOutput output) override;

    std::vector<OperationInput> takeOperationCommands();
    // 명령을 순서대로 반영하고, 그중 ON으로 전환된 명령이 있으면 true를 반환한다.
    bool applyOperationCommands(const std::vector<OperationInput>& commands);

    MessageQueue<OperationInput>& _operationQueue;
    MessageQueue<BeamInput>& _beamQueue;
    MessageQueue<TargetInput>& _targetQueue;
    std::shared_ptr<spdlog::logger> _operationLogger;
    OperationStateGate _operationState;
    std::optional<std::chrono::steady_clock::time_point> _started;
    std::chrono::steady_clock::time_point _nextTick;
};
} // namespace app
