#pragma once

#include <spdlog/async_logger.h>

#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace app
{
class LogManager
{
public:
    virtual ~LogManager() noexcept;
    LogManager(const LogManager&) = delete;
    LogManager& operator=(const LogManager&) = delete;

protected:
    explicit LogManager(std::size_t queueSize);

    // 설정 작업은 공통 등록 절차 안에서만 실행하고 보관하지 않는다.
    void addLogger(
        const std::string& name,
        spdlog::level::level_enum level,
        spdlog::async_overflow_policy policy,
        const std::function<void(spdlog::logger&)>& configure,
        const std::function<void()>& onRegistered = {}
    );

private:
    void shutdown();

    std::mutex _mutex;
    std::shared_ptr<spdlog::details::thread_pool> _threadPool;
    std::vector<std::shared_ptr<spdlog::logger>> _loggers;
};
}
