#pragma once

#include "log/log_manager.hpp"

#include <cstddef>
#include <string>

namespace app
{

class MessageLogManager final : public LogManager
{
public:
    MessageLogManager(
        const std::string& logPath,
        std::size_t queueSize
    );
    
    // 모든 로거는 같은 메시지 로그 파일에 기록한다.
    // consoleLevel이 off가 아니면 그 레벨 이상의 메시지를 콘솔(stdout)에도 출력한다. 기본은 콘솔 출력 없음.
    // 파일과 콘솔 모두 level 미만의 메시지는 기록하지 않는다.
    std::shared_ptr<spdlog::logger> addLogger(
        const std::string& name,
        spdlog::level::level_enum level = spdlog::level::info,
        spdlog::async_overflow_policy policy = spdlog::async_overflow_policy::discard_new,
        spdlog::level::level_enum consoleLevel = spdlog::level::off
    );

private:
    void configureLogger(
        spdlog::logger& logger,
        spdlog::level::level_enum consoleLevel
    ) const;

    spdlog::sink_ptr _sink;
    std::string _logPath;
};

} // namespace app
