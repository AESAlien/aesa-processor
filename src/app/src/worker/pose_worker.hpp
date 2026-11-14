#pragma once

#include "dto/beam_input.hpp"
#include "dto/console_input.hpp"
#include "dto/pose_input.hpp"
#include "queue/message_queue.hpp"
#include "worker/worker.hpp"

#include <pose/controller/pose_controller.hpp>

#include <optional>

namespace app
{
using PoseOutput = pose::SetRadarPoseResult;

class PoseWorker final : public Worker<PoseInput, PoseOutput>
{
public:
    PoseWorker(
        const pose::PoseController& controller,
        MessageQueue<PoseInput>& poseQueue,
        MessageQueue<BeamInput>& beamQueue,
        MessageQueue<ConsoleInput>& consoleQueue,
        std::shared_ptr<spdlog::logger> logger
    );

private:
    PoseInput readInput() override;
    std::optional<PoseOutput> process(PoseInput input) override;
    void writeOutput(PoseOutput output) override;

    const pose::PoseController& _controller;
    MessageQueue<PoseInput>& _poseQueue;
    MessageQueue<BeamInput>& _beamQueue;
    MessageQueue<ConsoleInput>& _consoleQueue;
};
}
