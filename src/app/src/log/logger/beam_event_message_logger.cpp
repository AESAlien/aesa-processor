#include "beam_event_message_logger.hpp"

#include <beam/dto/request_beam_result.hpp>
#include <beam/dto/set_attitude_command.hpp>

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

void BeamEventMessageLogger::logAttitudeApplied(const beam::SetAttitudeCommand& command) noexcept
{
    try
    {
        if (const auto logger = spdlog::get(LOGGER_NAME))
        {
            logger->log(
                spdlog::level::info,
                "Attitude applied: roll={}, pitch={}, yaw={}",
                command.roll.deg(),
                command.pitch.deg(),
                command.yaw.deg()
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

void BeamEventMessageLogger::logAttitudeIgnoredWhileOff(const beam::SetAttitudeCommand& command) noexcept
{
    // ON 전에 받은 자세정보는 버려지므로, ON 이후 자세정보가 다시 오지 않으면 탐색 빔이 나가지 않는다.
    try
    {
        if (const auto logger = spdlog::get(LOGGER_NAME))
        {
            logger->log(
                spdlog::level::warn,
                "Attitude ignored (operation state is OFF): roll={}, pitch={}, yaw={}",
                command.roll.deg(),
                command.pitch.deg(),
                command.yaw.deg()
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

void BeamEventMessageLogger::logAttitudeIgnoredAlreadySet(const beam::SetAttitudeCommand& command) noexcept
{
    // 운용 중 다시 받은 자세정보는 반영되지 않고 기존 자세정보를 계속 쓴다.
    try
    {
        if (const auto logger = spdlog::get(LOGGER_NAME))
        {
            logger->log(
                spdlog::level::warn,
                "Attitude ignored (already set): roll={}, pitch={}, yaw={}",
                command.roll.deg(),
                command.pitch.deg(),
                command.yaw.deg()
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
