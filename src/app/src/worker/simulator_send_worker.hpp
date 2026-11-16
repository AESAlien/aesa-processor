#pragma once

#include "worker/worker.hpp"

#include <optional>

#include "dto/simulator_input.hpp"
#include "queue/message_queue.hpp"
#include <comm/controller/comm_controller.hpp>
#include <variant>

namespace app
{
using SimulatorSendOutput = std::monostate;

class SimulatorSendWorker final : public Worker<SimulatorInput, std::monostate>
{
public:
    SimulatorSendWorker(
        comm::CommController& controller,
        MessageQueue<SimulatorInput>& simulatorQueue,
        std::shared_ptr<spdlog::logger> logger
    );

private:
    SimulatorInput readInput() override;
    std::optional<SimulatorSendOutput> process(SimulatorInput input) override;

    comm::CommController& _controller;
    MessageQueue<SimulatorInput>& _simulatorQueue;
};
} // namespace app
