#pragma once

#include "worker/worker.hpp"

#include "dto/target_input.hpp"
#include "dto/beam_input.hpp"
#include "dto/console_input.hpp"
#include "queue/message_queue.hpp"
#include <target/controller/target_controller.hpp>
#include <vector>
#include <beam/dto/request_beam_command.hpp>
#include <target/dto/track_event.hpp>
#include <variant>
#include <optional>

namespace app
{
using TargetOutput = std::variant<
    beam::RequestBeamCommand,
    std::vector<target::TrackEvent>
>;

class TargetWorker final : public Worker<TargetInput, std::optional<TargetOutput>>
{
public:
    TargetWorker(
        target::TargetController& controller,
        MessageQueue<TargetInput>& targetQueue,
        MessageQueue<BeamInput>& beamQueue,
        MessageQueue<ConsoleInput>& consoleQueue,
        std::shared_ptr<spdlog::logger> logger
    );

private:
    TargetInput readInput() override;
    std::optional<TargetOutput> process(TargetInput input) override;
    void writeOutput(std::optional<TargetOutput> output) override;

    target::TargetController& _controller;
    MessageQueue<TargetInput>& _targetQueue;
    MessageQueue<BeamInput>& _beamQueue;
    MessageQueue<ConsoleInput>& _consoleQueue;
};
} // namespace app
