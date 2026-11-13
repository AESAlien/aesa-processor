#pragma once

#include "worker/worker.hpp"

#include "dto/beam_input.hpp"
#include "dto/target_input.hpp"
#include "dto/console_input.hpp"
#include "queue/message_queue.hpp"
#include <comm/controller/comm_controller.hpp>
#include <comm/dto/incoming_dto.hpp>
#include <optional>

namespace app
{
class CommReceiveWorker final : public Worker<std::optional<comm::IncomingDto>, std::optional<comm::IncomingDto>>
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
    std::optional<comm::IncomingDto> readInput() override;
    std::optional<comm::IncomingDto> process(std::optional<comm::IncomingDto> input) override;
    void writeOutput(std::optional<comm::IncomingDto> output) override;

    comm::CommController& _controller;
    MessageQueue<BeamInput>& _beamQueue;
    MessageQueue<TargetInput>& _targetQueue;
    MessageQueue<ConsoleInput>& _consoleQueue;
};
} // namespace app
