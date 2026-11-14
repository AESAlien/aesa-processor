#pragma once

#include "worker/worker.hpp"

#include "dto/beam_input.hpp"
#include "dto/simulator_input.hpp"
#include "queue/message_queue.hpp"
#include <beam/controller/beam_controller.hpp>
#include <optional>

namespace app
{
using BeamOutput = std::optional<beam::TransmitBeamCommand>;

class BeamWorker final : public Worker<BeamInput, BeamOutput>
{
public:
    BeamWorker(
        beam::BeamController& controller,
        MessageQueue<BeamInput>& beamQueue,
        MessageQueue<SimulatorInput>& simulatorQueue,
        std::shared_ptr<spdlog::logger> logger
    );

private:
    BeamInput readInput() override;
    BeamOutput process(BeamInput input) override;
    void writeOutput(BeamOutput output) override;

    beam::BeamController& _controller;
    MessageQueue<BeamInput>& _beamQueue;
    MessageQueue<SimulatorInput>& _simulatorQueue;
};
} // namespace app
