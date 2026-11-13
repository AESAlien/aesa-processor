#pragma once

#include "worker/worker.hpp"

#include "dto/simulator_input.hpp"
#include "queue/message_queue.hpp"
#include <comm/controller/comm_controller.hpp>
#include <variant>

namespace app
{
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
    std::monostate process(SimulatorInput input) override;

    comm::CommController& _controller;
    MessageQueue<SimulatorInput>& _simulatorQueue;
};
} // namespace app
