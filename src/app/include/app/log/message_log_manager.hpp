#pragma once

#include <app/log/log_manager.hpp>

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
    
    void addLogger(
        const std::string& name,
        spdlog::level::level_enum level = spdlog::level::info,
        spdlog::async_overflow_policy policy = spdlog::async_overflow_policy::discard_new
    );

private:
    void configureLogger(spdlog::logger& logger) const;

    spdlog::sink_ptr _sink;
    std::string _logPath;
};

} // namespace app
