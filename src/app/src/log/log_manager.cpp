#include <app/log/log_manager.hpp>

#include <spdlog/details/thread_pool.h>
#include <spdlog/spdlog.h>

#include <stdexcept>

namespace app
{
LogManager::LogManager(std::size_t queueSize)
{
    if (queueSize == 0)
    {
        throw std::invalid_argument("Logging queue size must be positive");
    }
    _threadPool = std::make_shared<spdlog::details::thread_pool>(queueSize, 1);
}

LogManager::~LogManager() noexcept
{
    shutdown();
}

void LogManager::addLogger(
    const std::string& name,
    spdlog::level::level_enum level,
    spdlog::async_overflow_policy policy,
    const std::function<void(spdlog::logger&)>& configure,
    const std::function<void()>& onRegistered
) {
    std::lock_guard<std::mutex> lock(_mutex);

    if (name.empty())
    {
        throw std::invalid_argument("Logger name cannot be empty");
    }

    auto logger = std::make_shared<spdlog::async_logger>(
        name,
        spdlog::sinks_init_list{},
        _threadPool,
        policy
    );
    logger->set_level(level);
    configure(*logger);

    _loggers.push_back(logger);
    try
    {
        spdlog::register_logger(logger);
    }
    catch (...)
    {
        _loggers.pop_back();
        throw;
    }

    try
    {
        if (onRegistered)
        {
            onRegistered();
        }
    }
    catch (...)
    {
        if (spdlog::get(logger->name()) == logger)
        {
            spdlog::drop(logger->name());
        }
        _loggers.pop_back();
        throw;
    }
}

void LogManager::shutdown()
{
    std::lock_guard<std::mutex> lock(_mutex);

    for (const auto& logger : _loggers)
    {
        logger->flush();
        if (spdlog::get(logger->name()) == logger)
        {
            spdlog::drop(logger->name());
        }
    }

    // 하위 클래스의 상태를 사용하지 않고 대기 중인 기록을 모두 완료한다.
    _threadPool.reset();
    _loggers.clear();
}
}
