#include "operation/operation_state_gate.hpp"

#include <gtest/gtest.h>

namespace app
{
namespace
{
using beam::OperationState;

// 운용 명령을 받기 전에는 OFF로 시작해 빔 명령과 새 트랙 생성을 막는다.
TEST(OperationStateGateTest, StartsOff)
{
    const OperationStateGate gate;

    EXPECT_FALSE(gate.isOn());
}

// 처음 상태와 같은 OFF는 전환이 아니므로 초기화 동작을 일으키지 않는다.
TEST(OperationStateGateTest, InitialOffIsNotATransition)
{
    OperationStateGate gate;

    EXPECT_FALSE(gate.apply(OperationState::OFF));
    EXPECT_FALSE(gate.isOn());
}

// 같은 명령을 반복 수신하면 첫 번째만 전환으로 판단한다.
TEST(OperationStateGateTest, RepeatedCommandsChangeStateOnlyOnce)
{
    OperationStateGate gate;

    EXPECT_TRUE(gate.apply(OperationState::ON));
    EXPECT_FALSE(gate.apply(OperationState::ON));
    EXPECT_FALSE(gate.apply(OperationState::ON));
    EXPECT_TRUE(gate.isOn());

    EXPECT_TRUE(gate.apply(OperationState::OFF));
    EXPECT_FALSE(gate.apply(OperationState::OFF));
    EXPECT_FALSE(gate.isOn());
}

// 연속 전환에서도 매번 전환으로 판단하고 마지막 명령의 상태를 따른다.
TEST(OperationStateGateTest, AlternatingCommandsChangeStateEveryTime)
{
    OperationStateGate gate;

    for (int cycle = 0; cycle < 3; ++cycle)
    {
        EXPECT_TRUE(gate.apply(OperationState::ON));
        EXPECT_TRUE(gate.isOn());
        EXPECT_TRUE(gate.apply(OperationState::OFF));
        EXPECT_FALSE(gate.isOn());
    }
}

} // namespace
} // namespace app
