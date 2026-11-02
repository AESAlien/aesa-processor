#include <app/worker/worker_error.hpp>

#include <spdlog/spdlog.h>

#include <cstdio>

namespace app
{

// Select logger
// reason : comm 3 workers.
const char* workerModule(
    std::string_view worker) noexcept
{
    if (worker.find("Beam") == 0)
    {
        return "beam";
    }

    if (worker.find("Target") == 0)
    {
        return "target";
    }

    if (worker.find("Comm") == 0)
    {
        return "comm";
    }

    return "app";
}

void reportWorkerError(
    const char* worker,
    const char* message) noexcept
{
    try
    {
        if (auto logger = spdlog::get(workerModule(worker)))
        {
            logger->error(
                "[worker={}] {}",
                worker,
                message);

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