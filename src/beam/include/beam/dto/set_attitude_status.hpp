#pragma once

#include <cstdint>

namespace beam
{
// 자세정보(BeamController::setAttitude)의 처리 상태.
enum class SetAttitudeStatus : uint8_t
{
    // 자세정보를 반영했다. 이후 탐색 격자와 추적 영역 판정에 쓰인다.
    APPLIED,
    // 운용 상태가 ON이 아니어서 무시했다.
    IGNORED_NOT_ON,
    // 이미 자세정보를 받은 상태여서 무시했다. 자세정보는 ON마다 한 번만 반영한다.
    IGNORED_ALREADY_SET
};

} // namespace beam
