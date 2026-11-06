#pragma once

#include <spdlog/async.h>
#include <spdlog/sinks/sink.h>

#include <cstddef>
#include <mutex>
#include <string>

namespace logging
{

class LogManager
{
public:
    LogManager() = default;

    ~LogManager() noexcept;

    LogManager(const LogManager&) = delete;
    LogManager& operator=(const LogManager&) = delete;

    void initialize(
        const std::string& logPath,
        const std::string& errorPath,
        std::size_t queueSize);

    void addLogger(
        const std::string& name,
        spdlog::level::level_enum level =
            spdlog::level::info,
        spdlog::async_overflow_policy policy =
            spdlog::async_overflow_policy::discard_new);

    void shutdown();

private:
    std::mutex _mutex;
    spdlog::sink_ptr _sink;
    bool _initialized = false;
};

} // namespace logging