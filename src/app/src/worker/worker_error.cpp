#include <app/worker/worker_error.hpp>

#include <spdlog/spdlog.h>

#include <cstdio>

namespace app
{

void reportWorkerError(
    const char* worker,
    const char* message) noexcept
{
    try
    {
        // main에서 등록한 워커 이름으로 직접 조회한다.
        if (auto logger = spdlog::get(worker))
        {
            // 출력 패턴의 %n에 워커 이름이 포함된다.
            logger->error("{}", message);

            return;
        }
    }
    catch (...)
    {
    }

    // 로거를 사용할 수 없으면 콘솔로 출력한다.
    std::fprintf(
        stderr,
        "[%s] %s\n",
        worker,
        message);
}

} // namespace app