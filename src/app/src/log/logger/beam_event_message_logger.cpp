#include "beam_event_message_logger.hpp"

#include <beam/dto/request_beam_result.hpp>

#include <magic_enum/magic_enum.hpp>
#include <spdlog/spdlog.h>

#include <cstdio>
#include <exception>

namespace app
{
namespace
{
// OperationState는 bool 기반 enum이므로 magic_enum 대신 직접 변환한다.
const char* operationStateName(beam::OperationState state)
{
    return state == beam::OperationState::ON ? "ON" : "OFF";
}
}

void BeamEventMessageLogger::logOperationStateChanged(beam::OperationState state) noexcept
{
    try
    {
        if (const auto logger = spdlog::get(LOGGER_NAME))
        {
            logger->log(spdlog::level::info, "Operation state changed: {}", operationStateName(state));
        }
    }
    catch (const std::exception& error)
    {
        std::fprintf(stderr, "[BeamEventMessageLogger] %s\n", error.what());
    }
    catch (...)
    {
        std::fprintf(stderr, "[BeamEventMessageLogger] Logging failed\n");
    }
}

void BeamEventMessageLogger::logAttitudeApplied() noexcept
{
    try
    {
        if (const auto logger = spdlog::get(LOGGER_NAME))
        {
            logger->info("Attitude applied");
        }
    }
    catch (const std::exception& error)
    {
        std::fprintf(stderr, "[BeamEventMessageLogger] %s\n", error.what());
    }
    catch (...)
    {
        std::fprintf(stderr, "[BeamEventMessageLogger] Logging failed\n");
    }
}

void BeamEventMessageLogger::logRequestBeamRejected(const beam::RequestBeamResult& result) noexcept
{
    try
    {
        if (const auto logger = spdlog::get(LOGGER_NAME))
        {
            logger->log(
                spdlog::level::warn,
                "Beam request rejected: status={}, requestId={}",
                magic_enum::enum_name(result.status),
                result.requestId
            );
        }
    }
    catch (const std::exception& error)
    {
        std::fprintf(stderr, "[BeamEventMessageLogger] %s\n", error.what());
    }
    catch (...)
    {
        std::fprintf(stderr, "[BeamEventMessageLogger] Logging failed\n");
    }
}
}
