#include "worker/tick_worker.hpp"

#include <gtest/gtest.h>
#include <spdlog/sinks/null_sink.h>

#include <chrono>
#include <memory>
#include <optional>
#include <variant>

namespace app
{
namespace
{
using namespace std::chrono_literals;
using beam::OperationState;
using Time = std::chrono::milliseconds;

// ON 전에 이 횟수만큼 틱을 보내 시각을 충분히 늘린다.
constexpr int TICKS_BEFORE_ON = 10;
// 다시 센 시각과 이어서 센 시각을 구분하는 기준. 실행 지연을 감안해 넉넉히 잡는다.
constexpr Time RESET_TIME_LIMIT = 50ms;

std::shared_ptr<spdlog::logger> nullLogger()
{
    return std::make_shared<spdlog::logger>("tick_worker_test", std::make_shared<spdlog::sinks::null_sink_mt>());
}

class TickWorkerTest : public ::testing::Test
{
protected:
    // 틱 하나를 만들게 하고, 빔 큐에 쓴 틱의 시각을 반환한다.
    Time nextTickTime()
    {
        _worker.runOnce();
        const Time beamTime = std::get<TickEvent>(_beamQueue.read()).currentTime;
        const Time targetTime = std::get<TickEvent>(_targetQueue.read()).currentTime;
        EXPECT_EQ(beamTime, targetTime);
        return beamTime;
    }

    // 수신 워커처럼 운용 명령을 틱 워커의 큐에 넣는다.
    void sendOperation(OperationState state)
    {
        _operationQueue.write(OperationInput(state));
    }

    // ON을 반영시키고, 두 큐에 쓴 ON 명령과 틱을 꺼낸다.
    void turnOn()
    {
        sendOperation(OperationState::ON);
        _worker.runOnce();
        for (int index = 0; index < 2; ++index)
        {
            _beamQueue.read();
            _targetQueue.read();
        }
    }

    template <typename Input>
    static std::optional<OperationState> operationStateOf(const Input& message)
    {
        if (const auto* command = std::get_if<beam::SetOperationStateCommand>(&message))
        {
            return command->powerState;
        }
        return std::nullopt;
    }

    MessageQueue<OperationInput> _operationQueue;
    MessageQueue<BeamInput> _beamQueue;
    MessageQueue<TargetInput> _targetQueue;
    TickWorker _worker{_operationQueue, _beamQueue, _targetQueue, nullLogger(), nullptr};
};

// 운용 명령이 없으면 틱 시각은 실행 시작부터 계속 늘어난다.
TEST_F(TickWorkerTest, TickTimeIncreasesWithoutOperationCommand)
{
    Time previous = nextTickTime();
    for (int index = 0; index < 3; ++index)
    {
        const Time current = nextTickTime();
        EXPECT_GT(current, previous);
        previous = current;
    }
}

// ON으로 전환되면 두 큐 모두 ON 명령 다음에 시각 0부터 다시 센 틱을 받는다.
TEST_F(TickWorkerTest, TurningOnResetsTickTimeAfterForwardingCommand)
{
    Time beforeOn{0};
    for (int index = 0; index < TICKS_BEFORE_ON; ++index)
    {
        beforeOn = nextTickTime();
    }
    ASSERT_GE(beforeOn, RESET_TIME_LIMIT);

    sendOperation(OperationState::ON);
    _worker.runOnce();

    EXPECT_EQ(operationStateOf(_beamQueue.read()), OperationState::ON);
    EXPECT_EQ(operationStateOf(_targetQueue.read()), OperationState::ON);
    const Time beamTime = std::get<TickEvent>(_beamQueue.read()).currentTime;
    const Time targetTime = std::get<TickEvent>(_targetQueue.read()).currentTime;
    EXPECT_EQ(beamTime, 0ms);
    EXPECT_EQ(targetTime, 0ms);

    // 다시 센 시각도 10ms 간격으로 늘어난다.
    const Time next = nextTickTime();
    EXPECT_GE(next, 10ms);
    EXPECT_LT(next, RESET_TIME_LIMIT);
}

// OFF로 전환되어도 시각은 이어서 늘어난다.
TEST_F(TickWorkerTest, TurningOffKeepsTickTime)
{
    turnOn();
    const Time beforeOff = nextTickTime();

    sendOperation(OperationState::OFF);
    _worker.runOnce();
    EXPECT_EQ(operationStateOf(_beamQueue.read()), OperationState::OFF);
    EXPECT_EQ(operationStateOf(_targetQueue.read()), OperationState::OFF);
    EXPECT_GT(std::get<TickEvent>(_beamQueue.read()).currentTime, beforeOff);
    EXPECT_GT(std::get<TickEvent>(_targetQueue.read()).currentTime, beforeOff);
}

// 같은 ON을 반복해서 받으면 명령은 전달하지만 시각은 다시 세지 않는다.
TEST_F(TickWorkerTest, RepeatedOnDoesNotResetTickTime)
{
    turnOn();
    const Time afterOn = nextTickTime();

    sendOperation(OperationState::ON);
    _worker.runOnce();
    EXPECT_EQ(operationStateOf(_beamQueue.read()), OperationState::ON);
    EXPECT_EQ(operationStateOf(_targetQueue.read()), OperationState::ON);
    EXPECT_GT(std::get<TickEvent>(_beamQueue.read()).currentTime, afterOn);
    EXPECT_GT(std::get<TickEvent>(_targetQueue.read()).currentTime, afterOn);
}

// 한 틱 사이에 OFF와 ON을 모두 받으면 받은 순서대로 전달하고, 시각은 다시 센다.
TEST_F(TickWorkerTest, ForwardsCommandsInOrderAndResetsWhenTurnedOffAndOnWithinOneTick)
{
    turnOn();
    for (int index = 0; index < TICKS_BEFORE_ON; ++index)
    {
        nextTickTime();
    }

    sendOperation(OperationState::OFF);
    sendOperation(OperationState::ON);
    _worker.runOnce();

    EXPECT_EQ(operationStateOf(_beamQueue.read()), OperationState::OFF);
    EXPECT_EQ(operationStateOf(_beamQueue.read()), OperationState::ON);
    EXPECT_EQ(std::get<TickEvent>(_beamQueue.read()).currentTime, 0ms);
    EXPECT_EQ(operationStateOf(_targetQueue.read()), OperationState::OFF);
    EXPECT_EQ(operationStateOf(_targetQueue.read()), OperationState::ON);
    EXPECT_EQ(std::get<TickEvent>(_targetQueue.read()).currentTime, 0ms);
}

} // namespace
} // namespace app
