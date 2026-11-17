#pragma once

#include <beam/domain/operation_state.hpp>

namespace app
{
// 운용 상태를 보유하고 상태 전환 여부를 판단한다. 처음 상태는 OFF다.
// 운용 명령을 받는 워커마다 하나씩 두며, 워커 스레드에서만 호출하므로 스레드 안전하지 않다.
class OperationStateGate
{
public:
    // 상태를 바꾸고, 실제로 바뀌었으면 true를 반환한다.
    // 현재와 같은 상태가 들어오면 아무것도 바꾸지 않고 false를 반환한다.
    bool apply(beam::OperationState state);

    bool isOn() const;

private:
    beam::OperationState _state = beam::OperationState::OFF;
};

} // namespace app
