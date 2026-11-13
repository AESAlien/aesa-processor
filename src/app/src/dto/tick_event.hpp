#pragma once

#include <chrono>

namespace app
{

struct TickEvent
{
    // 실행 시작부터 경과한 시간을 밀리초 단위로 표현한다.
    std::chrono::milliseconds currentTime{0};
};

} // namespace app
