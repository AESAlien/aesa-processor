#pragma once

#include <cstdint>

namespace beam
{
// 운용 상태 변경 명령(BeamScheduler::setOperationState)의 처리 상태.
enum class SetOperationStateStatus : uint8_t
{
    CHANGED,
    UNCHANGED
};

} // namespace beam
