#pragma once

#include "worker/worker.hpp"

#include "dto/target_input.hpp"
#include "dto/beam_input.hpp"
#include "dto/console_input.hpp"
#include "operation/operation_state_gate.hpp"
#include "queue/message_queue.hpp"
#include <target/controller/target_controller.hpp>
#include <vector>
#include <beam/dto/request_beam_command.hpp>
#include <target/dto/track_event.hpp>
#include <chrono>
#include <variant>
#include <optional>

namespace app
{
using TargetOutput = std::variant<
    beam::RequestBeamCommand,
    std::vector<target::TrackEvent>
>;
class TargetWorker final : public Worker<TargetInput, TargetOutput>
{
public:
    TargetWorker(
        target::TargetController& controller,
        MessageQueue<TargetInput>& targetQueue,
        MessageQueue<BeamInput>& beamQueue,
        MessageQueue<ConsoleInput>& consoleQueue,
        std::shared_ptr<spdlog::logger> logger,
        std::shared_ptr<spdlog::logger> operationLogger
    );

private:
    TargetInput readInput() override;
    std::optional<TargetOutput> process(TargetInput input) override;
    void writeOutput(TargetOutput output) override;
    void applyOperationState(beam::OperationState state);

    target::TargetController& _controller;
    MessageQueue<TargetInput>& _targetQueue;
    MessageQueue<BeamInput>& _beamQueue;
    MessageQueue<ConsoleInput>& _consoleQueue;
    std::shared_ptr<spdlog::logger> _operationLogger;
    OperationStateGate _operationState;
    // 마지막으로 받은 틱의 시각. ON으로 전환되면 틱 워커가 시각을 0부터 다시 세므로 0으로 맞춘다.
    std::chrono::milliseconds _currentTime{0};
};
} // namespace app
