#pragma once

#include <chrono>

namespace app
{

struct TickEvent
{
    // 마지막 운용 ON 전환부터 경과한 시간을 밀리초 단위로 표현한다.
    // 처음 ON이 되기 전에는 실행 시작부터 경과한 시간이다.
    std::chrono::milliseconds currentTime{0};
};

} // namespace app
