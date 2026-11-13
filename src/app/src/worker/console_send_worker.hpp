#pragma once

#include "worker/worker.hpp"

#include "dto/console_input.hpp"
#include "queue/message_queue.hpp"
#include <comm/controller/comm_controller.hpp>
#include <variant>

namespace app
{
class ConsoleSendWorker final : public Worker<ConsoleInput, std::monostate>
{
public:
    ConsoleSendWorker(
        comm::CommController& controller,
        MessageQueue<ConsoleInput>& consoleQueue,
        std::shared_ptr<spdlog::logger> logger
    );

private:
    ConsoleInput readInput() override;
    std::monostate process(ConsoleInput input) override;

    comm::CommController& _controller;
    MessageQueue<ConsoleInput>& _consoleQueue;
};
} // namespace app
