#pragma once

#include <cstdint>

namespace beam
{

// 빔 종류. 값은 시나리오 모의기와 주고받는 메시지의 beamType 값과 같다.
enum class BeamType : uint8_t
{
    SEARCH = 1,
    CONFIRMATION = 2,
    TRACKING = 3
};

// 송신을 요청할 수 있는 빔 종류. BeamType의 부분집합이며 같은 값을 가진다.
enum class RequestBeamType : uint8_t
{
    CONFIRMATION = 2,
    TRACKING = 3
};

static_assert(
    static_cast<uint8_t>(RequestBeamType::CONFIRMATION) == static_cast<uint8_t>(BeamType::CONFIRMATION),
    "RequestBeamType::CONFIRMATION must have the same value as BeamType::CONFIRMATION");
static_assert(
    static_cast<uint8_t>(RequestBeamType::TRACKING) == static_cast<uint8_t>(BeamType::TRACKING),
    "RequestBeamType::TRACKING must have the same value as BeamType::TRACKING");

} // namespace beam
