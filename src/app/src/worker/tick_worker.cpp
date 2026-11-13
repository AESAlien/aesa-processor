#include "worker/tick_worker.hpp"

#include <thread>
#include <utility>

namespace app
{
namespace
{
constexpr std::chrono::milliseconds TICK_PERIOD{10};
}

TickWorker::TickWorker(
    MessageQueue<BeamInput>& beamQueue,
    MessageQueue<TargetInput>& targetQueue,
    std::shared_ptr<spdlog::logger> logger
) : Worker(std::move(logger)),
    _beamQueue(beamQueue),
    _targetQueue(targetQueue)
{
}

std::monostate TickWorker::readInput()
{
    return {};
}

TickEvent TickWorker::process(std::monostate)
{
    if (!_started)
    {
        _started = std::chrono::steady_clock::now();
        _nextTick = *_started;
    }
    std::this_thread::sleep_until(_nextTick);
    const auto now = std::chrono::steady_clock::now();
    const auto currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - *_started
    );
    _nextTick += TICK_PERIOD;
    if (_nextTick <= now)
    {
        _nextTick = now + TICK_PERIOD;
    }
    return TickEvent{currentTime};
}

void TickWorker::writeOutput(TickEvent output)
{
    _beamQueue.write(output);
    _targetQueue.write(std::move(output));
}
} // namespace app
