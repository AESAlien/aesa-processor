#pragma once

#include <cstdint>

namespace beam
{
// 운용 상태 변경 명령(BeamScheduler::setOperationState)의 처리 상태.
enum class SetOperationStateStatus : uint8_t
{
    // 운용 상태가 바뀌었다. OFF에서 ON으로 바뀌면 모든 상태(요청 큐, 자세정보)가 초기화된다.
    CHANGED,
    // 현재와 같은 상태가 들어와 아무것도 바꾸지 않았다.
    UNCHANGED
};

} // namespace beam
