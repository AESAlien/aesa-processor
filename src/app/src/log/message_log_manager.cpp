#include "log/message_log_manager.hpp"

#include "support/timestamped_log_path.hpp"

#include <spdlog/pattern_formatter.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/callback_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <cstdio>
#include <functional>
#include <memory>
#include <system_error>

namespace app
{
namespace
{
void errorHandler(const std::string& error)
{
    std::fprintf(stderr, "[MessageLogManager] %s\n", error.c_str());
}

void messageHandler(
    const spdlog::sink_ptr& sink,
    const spdlog::details::log_msg& message
) {
    sink->log(message);
    sink->flush();
}
}

MessageLogManager::MessageLogManager(
    const std::string& logPath,
    std::size_t queueSize
) : LogManager(queueSize),
    _logPath(reserveTimestampedLogPath(logPath).string())
{
    try
    {
        auto file = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
            _logPath,
            false
        );
        file->set_formatter(std::make_unique<spdlog::pattern_formatter>(
            "%Y-%m-%dT%H:%M:%S.%eZ "
            "[%n] [%l] [thread=%t] %v",
            spdlog::pattern_time_type::utc
        ));

        // 로그 스레드에서 모든 레벨의 메시지를 같은 파일에 기록한다.
        _sink = std::make_shared<spdlog::sinks::callback_sink_mt>(
            std::bind(messageHandler, file, std::placeholders::_1)
        );
    }
    catch (...)
    {
        std::error_code error;
        std::filesystem::remove(_logPath, error);
        throw;
    }
}

std::shared_ptr<spdlog::logger> MessageLogManager::addLogger(
    const std::string& name,
    spdlog::level::level_enum level,
    spdlog::async_overflow_policy policy,
    spdlog::level::level_enum consoleLevel
) {
    return LogManager::addLogger(
        name,
        level,
        policy,
        [this, consoleLevel](spdlog::logger& logger)
        {
            configureLogger(logger, consoleLevel);
        }
    );
}

void MessageLogManager::configureLogger(
    spdlog::logger& logger,
    spdlog::level::level_enum consoleLevel
) const {
    logger.sinks().push_back(_sink);

    if (consoleLevel != spdlog::level::off)
    {
        // 로거마다 콘솔 레벨이 달라 sink를 따로 만든다. 콘솔 sink끼리는 spdlog의 전역 콘솔 뮤텍스를 공유한다.
        auto console = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console->set_level(consoleLevel);
        console->set_formatter(std::make_unique<spdlog::pattern_formatter>(
            "%H:%M:%S.%e [%n] [%^%l%$] %v"
        ));
        logger.sinks().push_back(console);
    }

    logger.set_error_handler(errorHandler);
}

} // namespace app
