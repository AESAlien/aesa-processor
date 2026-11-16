#pragma once

#include "worker/worker.hpp"

#include <optional>

#include "dto/console_input.hpp"
#include "queue/message_queue.hpp"
#include <comm/controller/comm_controller.hpp>
#include <variant>

namespace app
{
using ConsoleSendOutput = std::monostate;

class ConsoleSendWorker final : public Worker<ConsoleInput, ConsoleSendOutput>
{
public:
    ConsoleSendWorker(
        comm::CommController& controller,
        MessageQueue<ConsoleInput>& consoleQueue,
        std::shared_ptr<spdlog::logger> logger
    );

private:
    ConsoleInput readInput() override;
    std::optional<ConsoleSendOutput> process(ConsoleInput input) override;

    comm::CommController& _controller;
    MessageQueue<ConsoleInput>& _consoleQueue;
};
} // namespace app
