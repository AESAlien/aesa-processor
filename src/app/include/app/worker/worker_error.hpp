#pragma once

#include <exception>
#include <string_view>
#include <utility>

namespace app
{

const char* workerModule(
    std::string_view worker) noexcept;

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