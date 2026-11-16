#pragma once

#include "worker/worker.hpp"

#include "dto/beam_input.hpp"
#include "dto/simulator_input.hpp"
#include "operation/operation_state_gate.hpp"
#include "queue/message_queue.hpp"
#include <beam/controller/beam_controller.hpp>
#include <optional>

namespace app
{
using BeamOutput = beam::TransmitBeamCommand;

class BeamWorker final : public Worker<BeamInput, BeamOutput>
{
public:
    BeamWorker(
        beam::BeamController& controller,
        MessageQueue<BeamInput>& beamQueue,
        MessageQueue<SimulatorInput>& simulatorQueue,
        std::shared_ptr<spdlog::logger> logger,
        std::shared_ptr<spdlog::logger> operationLogger
    );

private:
    BeamInput readInput() override;
    std::optional<BeamOutput> process(BeamInput input) override;
    void writeOutput(BeamOutput output) override;
    void applyOperationState(beam::OperationState state);

    beam::BeamController& _controller;
    MessageQueue<BeamInput>& _beamQueue;
    MessageQueue<SimulatorInput>& _simulatorQueue;
    std::shared_ptr<spdlog::logger> _operationLogger;
    OperationStateGate _operationState;
};
} // namespace app
