#pragma once

#include <chrono>

namespace comm
{

// 모니터에 데이터 처리부 상태를 보내도록 요청한다.
// 상대 연결 상태는 통신 모듈이 송신 시점에 채운다.
struct MonitorStatusRequest
{
    // 빔 송신 명령의 시각과 같은 기준의 처리부 시각
    std::chrono::milliseconds processorTime{0};
    // 실제 시간 대비 처리부 시각의 진행 배율
    double timeScale = 1.0;
    bool paused = false;
};

} // namespace comm
