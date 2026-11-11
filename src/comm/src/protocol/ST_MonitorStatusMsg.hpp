#pragma once

#include <protocol/ST_MsgHeader.hpp>

#include <cstdint>

namespace comm::protocol
{

#pragma pack(push, 1)
// 데이터 처리부의 상태를 모니터에 알린다.
struct ST_MonitorStatusMsg
{
    ST_MsgHeader msgHeader;
    uint32_t processorTime_ms;
    float timeScale;
    uint8_t paused;
    uint8_t operatorConsoleConnected;
    uint8_t scenarioSimulatorConnected;
    uint8_t padding;
};
#pragma pack(pop)

static_assert(sizeof(ST_MonitorStatusMsg) == 32, "ST_MonitorStatusMsg must be 32 bytes");

} // namespace comm::protocol
