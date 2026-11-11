#pragma once

#include <comm/dto/monitor_status_request.hpp>

namespace comm
{

// 모니터 상태 메시지로 인코딩할 값
struct MonitorStatus
{
    MonitorStatusRequest request;
    bool operatorConsoleConnected = false;
    bool scenarioSimulatorConnected = false;
};

} // namespace comm
