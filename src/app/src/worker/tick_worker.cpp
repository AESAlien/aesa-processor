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
    MessageQueue<OperationInput>& operationQueue,
    MessageQueue<BeamInput>& beamQueue,
    MessageQueue<TargetInput>& targetQueue,
    std::shared_ptr<spdlog::logger> logger,
    std::shared_ptr<spdlog::logger> operationLogger
) : Worker(std::move(logger)),
    _operationQueue(operationQueue),
    _beamQueue(beamQueue),
    _targetQueue(targetQueue),
    _operationLogger(std::move(operationLogger))
{
}

std::monostate TickWorker::readInput()
{
    return {};
}

std::optional<TickOutput> TickWorker::process(std::monostate)
{
    if (!_started)
    {
        _started = std::chrono::steady_clock::now();
        _nextTick = *_started;
    }
    std::this_thread::sleep_until(_nextTick);

    // 대기 중에 들어온 명령까지 이번 틱보다 먼저 전달한다.
    std::vector<OperationInput> commands = takeOperationCommands();
    const auto now = std::chrono::steady_clock::now();
    if (applyOperationCommands(commands))
    {
        // 모의기는 빔 명령의 시각으로 시나리오 위치를 계산하므로, ON마다 시나리오가 처음부터 진행되도록 0부터 센다.
        _started = now;
        _nextTick = now;
        if (_operationLogger)
        {
            _operationLogger->info("Tick time reset by operation ON");
        }
    }

    const auto currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - *_started
    );
    _nextTick += TICK_PERIOD;
    // 놓친 틱이 몰려 나가지 않도록 현재 시각을 기준으로 다음 틱을 예약한다.
    if (_nextTick <= now)
    {
        _nextTick = now + TICK_PERIOD;
    }
    return TickOutput{std::move(commands), TickEvent{currentTime}};
}

void TickWorker::writeOutput(TickOutput output)
{
    // 각 큐에서 운용 명령이 이번 틱보다 먼저 처리되도록 명령을 먼저 쓴다.
    for (const OperationInput& command : output.operationCommands)
    {
        _beamQueue.write(command);
        _targetQueue.write(command);
    }
    _beamQueue.write(output.tick);
    _targetQueue.write(output.tick);
}

std::vector<OperationInput> TickWorker::takeOperationCommands()
{
    std::vector<OperationInput> commands;
    while (std::optional<OperationInput> command = _operationQueue.tryRead())
    {
        commands.push_back(std::move(*command));
    }
    return commands;
}

bool TickWorker::applyOperationCommands(const std::vector<OperationInput>& commands)
{
    bool turnedOn = false;
    for (const OperationInput& command : commands)
    {
        // 같은 상태의 반복 수신은 전환이 아니므로 시간 기준을 바꾸지 않는다.
        if (_operationState.apply(command.powerState) &&
            command.powerState == beam::OperationState::ON)
        {
            turnedOn = true;
        }
    }
    return turnedOn;
}
} // namespace app
