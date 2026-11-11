#include "beam_event_message_logger.hpp"

#include <beam/dto/request_beam_result.hpp>
#include <beam/dto/set_attitude_command.hpp>

#include <magic_enum/magic_enum.hpp>
#include <spdlog/spdlog.h>

#include <cstdio>
#include <exception>
#include <utility>

namespace app
{
namespace
{
// OperationState는 bool 기반 enum이므로 magic_enum 대신 직접 변환한다.
const char* operationStateName(beam::OperationState state)
{
    return state == beam::OperationState::ON ? "ON" : "OFF";
}

// 로거가 등록되지 않았으면 기록하지 않는다. 기록 실패는 빔 처리에 전달하지 않는다.
template <typename... Args>
void write(
    spdlog::level::level_enum level,
    spdlog::format_string_t<Args...> format,
    Args&&... args
) noexcept {
    try
    {
        if (const auto logger = spdlog::get(BeamEventMessageLogger::LOGGER_NAME))
        {
            logger->log(level, format, std::forward<Args>(args)...);
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

void writeAttitude(
    spdlog::level::level_enum level,
    const char* event,
    const beam::SetAttitudeCommand& command
) noexcept {
    write(
        level,
        "Attitude {}: roll={}, pitch={}, yaw={}",
        event,
        command.roll.deg(),
        command.pitch.deg(),
        command.yaw.deg()
    );
}
}

void BeamEventMessageLogger::logOperationStateChanged(beam::OperationState state) noexcept
{
    write(spdlog::level::info, "Operation state changed: {}", operationStateName(state));
}

void BeamEventMessageLogger::logAttitudeApplied(const beam::SetAttitudeCommand& command) noexcept
{
    writeAttitude(spdlog::level::info, "applied", command);
}

void BeamEventMessageLogger::logAttitudeIgnoredWhileOff(const beam::SetAttitudeCommand& command) noexcept
{
    // ON 전에 받은 자세정보는 버려지므로, ON 이후 자세정보가 다시 오지 않으면 탐색 빔이 나가지 않는다.
    writeAttitude(spdlog::level::warn, "ignored (operation state is OFF)", command);
}

void BeamEventMessageLogger::logAttitudeIgnoredAlreadySet(const beam::SetAttitudeCommand& command) noexcept
{
    // 운용 중 다시 받은 자세정보는 반영되지 않고 기존 자세정보를 계속 쓴다.
    writeAttitude(spdlog::level::warn, "ignored (already set)", command);
}

void BeamEventMessageLogger::logRequestBeamRejected(const beam::RequestBeamResult& result) noexcept
{
    write(
        spdlog::level::warn,
        "Beam request rejected: status={}, requestId={}",
        magic_enum::enum_name(result.status),
        result.requestId
    );
}
}
