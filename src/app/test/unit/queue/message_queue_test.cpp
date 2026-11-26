#include "dto/beam_input.hpp"
#include "queue/message_queue.hpp"
#include "dto/target_input.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <stdexcept>
#include <type_traits>
#include <variant>

namespace app
{
namespace
{
// 운용 명령은 OFF 시 트랙을 지울 수 있도록 타겟 큐에도 전달된다.
static_assert(
    std::is_invocable_v<
        decltype(&MessageQueue<TargetInput>::write), MessageQueue<TargetInput>&, beam::SetOperationStateCommand
    >
);

TEST(MessageQueueTest, ReadsDifferentMessageTypesInWriteOrder)
{
    MessageQueue<BeamInput> input;
    input.write(beam::SetOperationStateCommand{beam::OperationState::ON});
    input.write(TickEvent{std::chrono::milliseconds{10}});
    input.write(beam::SetOperationStateCommand{beam::OperationState::OFF});

    EXPECT_EQ(std::get<beam::SetOperationStateCommand>(input.read()).powerState, beam::OperationState::ON);
    EXPECT_EQ(std::get<TickEvent>(input.read()).currentTime.count(), 10);
    EXPECT_EQ(std::get<beam::SetOperationStateCommand>(input.read()).powerState, beam::OperationState::OFF);
}

TEST(MessageQueueTest, TryReadReturnsMessagesInWriteOrderWithoutWaiting)
{
    MessageQueue<BeamInput> input;
    EXPECT_FALSE(input.tryRead().has_value());

    input.write(beam::SetOperationStateCommand{beam::OperationState::ON});
    input.write(TickEvent{std::chrono::milliseconds{10}});

    const auto first = input.tryRead();
    const auto second = input.tryRead();
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(std::get<beam::SetOperationStateCommand>(*first).powerState, beam::OperationState::ON);
    EXPECT_EQ(std::get<TickEvent>(*second).currentTime.count(), 10);
    EXPECT_FALSE(input.tryRead().has_value());
}

TEST(MessageQueueTest, ConcurrentProducersDoNotLoseOrReorderTheirMessages)
{
    MessageQueue<BeamInput> input;
    constexpr int MESSAGE_COUNT = 1000;
    auto ticks = std::async(
        std::launch::async,
        [&input]
        {
            for (int index = 0; index < MESSAGE_COUNT; ++index)
            {
                input.write(TickEvent{std::chrono::milliseconds{index}});
            }
        }
    );
    auto commands = std::async(
        std::launch::async,
        [&input]
        {
            for (int index = 0; index < MESSAGE_COUNT; ++index)
            {
                input.write(
                    beam::SetOperationStateCommand{
                        index % 2 == 0 ? beam::OperationState::ON : beam::OperationState::OFF
                    }
                );
            }
        }
    );

    int tickCount = 0;
    int commandCount = 0;
    for (int index = 0; index < MESSAGE_COUNT * 2; ++index)
    {
        const auto message = input.read();
        if (const auto* tick = std::get_if<TickEvent>(&message))
        {
            EXPECT_EQ(tick->currentTime.count(), tickCount++);
        }
        else
        {
            EXPECT_EQ(std::get<beam::SetOperationStateCommand>(message).powerState, commandCount % 2 == 0 ? beam::OperationState::ON : beam::OperationState::OFF);
            ++commandCount;
        }
    }
    ticks.get();
    commands.get();
    EXPECT_EQ(tickCount, MESSAGE_COUNT);
    EXPECT_EQ(commandCount, MESSAGE_COUNT);
}

TEST(MessageQueueTest, WriteWakesWaitingWorker)
{
    MessageQueue<BeamInput> input;
    auto received = std::async(
        std::launch::async,
        [&input]
        {
            return input.read();
        }
    );
    input.write(TickEvent{std::chrono::milliseconds{10}});

    const auto status = received.wait_for(std::chrono::seconds{2});
    if (status != std::future_status::ready)
    {
        input.close();
    }
    ASSERT_EQ(status, std::future_status::ready);
    EXPECT_EQ(std::get<TickEvent>(received.get()).currentTime.count(), 10);
}

TEST(MessageQueueTest, ClosingInputWakesReaderAndRejectsWrites)
{
    MessageQueue<BeamInput> input;
    auto received = std::async(
        std::launch::async,
        [&input]
        {
            return input.read();
        }
    );
    input.close();

    ASSERT_EQ(received.wait_for(std::chrono::seconds{2}), std::future_status::ready);
    EXPECT_THROW(received.get(), std::runtime_error);
    EXPECT_THROW(input.write(TickEvent{}), std::runtime_error);
    EXPECT_THROW(input.tryRead(), std::runtime_error);
}

TEST(MessageQueueTest, DifferentWorkersHaveIndependentInputs)
{
    MessageQueue<BeamInput> beamQueue;
    MessageQueue<TargetInput> targetQueue;
    beamQueue.write(TickEvent{std::chrono::milliseconds{10}});
    targetQueue.write(TickEvent{std::chrono::milliseconds{20}});
    beamQueue.close();

    EXPECT_THROW(beamQueue.read(), std::runtime_error);
    EXPECT_EQ(std::get<TickEvent>(targetQueue.read()).currentTime.count(), 20);
    targetQueue.write(TickEvent{std::chrono::milliseconds{30}});
    EXPECT_EQ(std::get<TickEvent>(targetQueue.read()).currentTime.count(), 30);
}
}
} // namespace app
