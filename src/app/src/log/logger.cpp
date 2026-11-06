#include <app/log/logger.hpp>

#include <spdlog/pattern_formatter.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/callback_sink.h>
#include <spdlog/spdlog.h>

#include <cstdio>
#include <memory>
#include <stdexcept>
#include <utility>

namespace logging
{

LogManager::~LogManager() noexcept
{
    try
    {
        shutdown();
    }
    catch (...)
    {
        std::fprintf(
            stderr,
            "[logging] Shutdown failed\n");
    }
}

void LogManager::initialize(
    const std::string& logPath,
    const std::string& errorPath,
    std::size_t queueSize)
{
    std::lock_guard<std::mutex> lock(_mutex);

    if (_initialized)
    {
        throw std::logic_error(
            "Logging can only be initialized once");
    }

    if (
        logPath.empty() ||
        errorPath.empty() ||
        logPath == errorPath ||
        queueSize == 0)
    {
        throw std::invalid_argument(
            "Invalid logging configuration");
    }

    auto normalFile =
        std::make_shared<spdlog::sinks::basic_file_sink_mt>(
            logPath,
            false);

    auto errorFile =
        std::make_shared<spdlog::sinks::basic_file_sink_mt>(
            errorPath,
            false);

    // 두 파일의 출력 형식을 동일하게 설정한다.
    for (const auto& file : {normalFile, errorFile})
    {
        file->set_formatter(
            std::make_unique<spdlog::pattern_formatter>(
                "%Y-%m-%dT%H:%M:%S.%eZ "
                "[%n] [%l] [thread=%t] %v",
                spdlog::pattern_time_type::utc));
    }

    // 로그 스레드에서 실행되며, 레벨에 따라 파일 하나를 선택한다.
    auto sink =
        std::make_shared<spdlog::sinks::callback_sink_mt>(
            [normalFile, errorFile](
                const spdlog::details::log_msg& message)
            {
                const auto& file =
                    message.level >= spdlog::level::err
                    ? errorFile
                    : normalFile;

                file->log(message);
                file->flush();
            });

    spdlog::init_thread_pool(queueSize, 1);

    _sink = std::move(sink);
    _initialized = true;
}

void LogManager::addLogger(
    const std::string& name,
    spdlog::level::level_enum level,
    spdlog::async_overflow_policy policy)
{
    std::lock_guard<std::mutex> lock(_mutex);

    if (!_sink)
    {
        throw std::logic_error(
            "Logging is not running");
    }

    auto logger =
        std::make_shared<spdlog::async_logger>(
            name,
            _sink,
            spdlog::thread_pool(),
            policy);

    logger->set_level(level);

    logger->set_error_handler(
        [](const std::string& error)
        {
            std::fprintf(
                stderr,
                "[logging] %s\n",
                error.c_str());
        });

    spdlog::register_logger(logger);
}

void LogManager::shutdown()
{
    std::lock_guard<std::mutex> lock(_mutex);

    if (!_sink)
    {
        return;
    }

    spdlog::shutdown();
    _sink.reset();
}

} // namespace logging