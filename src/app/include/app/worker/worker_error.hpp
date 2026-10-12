#pragma once

#include <exception>
#include <iostream>
#include <mutex>
#include <utility>

namespace app
{
inline void reportWorkerError(
    const char* worker,
    const char* message) noexcept
{
    try
    {
        static std::mutex mutex;
        std::lock_guard<std::mutex> lock(mutex);

        std::cerr << '[' << worker << "] "
                  << message << '\n';
    }
    catch (...)
    {
        // 로그 출력 중 발생한 예외는 전파하지 않는다.
    }
}

template <typename Function>
bool tryWorkerAction(const char* worker, Function&& action)
{
    try
    {
        std::forward<Function>(action)();
        return true;
    }
    catch (const std::exception& error)
    {
        reportWorkerError(worker, error.what());
    }
    catch (...)
    {
        reportWorkerError(worker, "Unknown exception");
    }

    return false;
}
} // namespace app