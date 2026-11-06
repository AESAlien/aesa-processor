#pragma once

#include <exception>
#include <utility>

namespace app
{

void reportWorkerError(
    const char* worker,
    const char* message) noexcept;

template <typename Function>
bool tryWorkerAction(
    const char* worker,
    Function&& action)
{
    try
    {
        std::forward<Function>(action)();

        return true;
    }
    catch (const std::exception& error)
    {
        reportWorkerError(
            worker,
            error.what());
    }
    catch (...)
    {
        reportWorkerError(
            worker,
            "Unknown exception");
    }

    return false;
}

} // namespace app