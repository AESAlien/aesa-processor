#pragma once

#include "worker/worker.hpp"

#include "dto/beam_input.hpp"
#include "dto/target_input.hpp"
#include "dto/console_input.hpp"
#include "queue/message_queue.hpp"
#include <comm/controller/comm_controller.hpp>
#include <comm/dto/incoming_dto.hpp>
#include <optional>
#include <variant>

namespace app
{
using CommReceiveOutput = comm::IncomingDto;

class CommReceiveWorker final : public Worker<std::monostate, CommReceiveOutput>
{
public:
    CommReceiveWorker(
        comm::CommController& controller,
        MessageQueue<BeamInput>& beamQueue,
        MessageQueue<TargetInput>& targetQueue,
        MessageQueue<ConsoleInput>& consoleQueue,
        std::shared_ptr<spdlog::logger> logger
    );

private:
    std::monostate readInput() override;
    std::optional<CommReceiveOutput> process(std::monostate input) override;
    void writeOutput(CommReceiveOutput output) override;

    comm::CommController& _controller;
    MessageQueue<BeamInput>& _beamQueue;
    MessageQueue<TargetInput>& _targetQueue;
    MessageQueue<ConsoleInput>& _consoleQueue;
};
} // namespace app
