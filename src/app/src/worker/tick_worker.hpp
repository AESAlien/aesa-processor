#pragma once

#include "worker/worker.hpp"

#include "dto/beam_input.hpp"
#include "dto/target_input.hpp"
#include "dto/tick_event.hpp"
#include "queue/message_queue.hpp"
#include <chrono>
#include <variant>

namespace app
{
using TickOutput = TickEvent;

class TickWorker final : public Worker<std::monostate, TickOutput>
{
public:
    TickWorker(
        MessageQueue<BeamInput>& beamQueue,
        MessageQueue<TargetInput>& targetQueue,
        std::shared_ptr<spdlog::logger> logger
    );

private:
    std::monostate readInput() override;
    TickOutput process(std::monostate input) override;
    void writeOutput(TickOutput output) override;

    MessageQueue<BeamInput>& _beamQueue;
    MessageQueue<TargetInput>& _targetQueue;
    std::optional<std::chrono::steady_clock::time_point> _started;
    std::chrono::steady_clock::time_point _nextTick;
};
} // namespace app
