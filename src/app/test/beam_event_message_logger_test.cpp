#include "logger/beam_event_message_logger.hpp"

#include <beam/dto/request_beam_result.hpp>
#include <beam/dto/set_attitude_command.hpp>

#include <spdlog/sinks/callback_sink.h>
#include <spdlog/spdlog.h>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace app
{
namespace
{
using namespace math::literals;

struct LoggedMessage
{
    spdlog::level::level_enum level;
    std::string text;
};

beam::SetAttitudeCommand attitude()
{
    return beam::SetAttitudeCommand::Builder()
        .latitude(37.0_deg)
        .longitude(127.0_deg)
        .altitude(0.1_km)
        .roll(1.5_deg)
        .pitch(-2.5_deg)
        .yaw(30_deg)
        .build();
}

class BeamEventMessageLoggerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        _logger = std::make_shared<spdlog::logger>(
            BeamEventMessageLogger::LOGGER_NAME,
            std::make_shared<spdlog::sinks::callback_sink_mt>([this](const spdlog::details::log_msg& message)
            {
                _messages.push_back({message.level, std::string(message.payload.data(), message.payload.size())});
            })
        );
        _logger->set_level(spdlog::level::trace);
        spdlog::register_logger(_logger);
    }

    void TearDown() override
    {
        spdlog::drop(BeamEventMessageLogger::LOGGER_NAME);
        _logger.reset();
    }

    BeamEventMessageLogger _beamLogger;
    std::shared_ptr<spdlog::logger> _logger;
    std::vector<LoggedMessage> _messages;
};

// 운용 상태 변경은 info로 기록한다.
TEST_F(BeamEventMessageLoggerTest, LogsOperationStateChangeAsInfo)
{
    // When: ON과 OFF 변경을 기록한다.
    _beamLogger.logOperationStateChanged(beam::OperationState::ON);
    _beamLogger.logOperationStateChanged(beam::OperationState::OFF);

    // Then: 상태 이름과 함께 info로 기록한다.
    ASSERT_EQ(_messages.size(), 2u);
    EXPECT_EQ(_messages[0].level, spdlog::level::info);
    EXPECT_EQ(_messages[0].text, "Operation state changed: ON");
    EXPECT_EQ(_messages[1].text, "Operation state changed: OFF");
}

// 자세정보 반영은 info, 무시는 warn으로 이유와 자세값을 담아 기록한다.
TEST_F(BeamEventMessageLoggerTest, LogsAttitudeAppliedAsInfoAndIgnoredAsWarn)
{
    // When: 반영, 무시(OFF), 무시(이미 받음)를 기록한다.
    _beamLogger.logAttitudeApplied(attitude());
    _beamLogger.logAttitudeIgnoredWhileOff(attitude());
    _beamLogger.logAttitudeIgnoredAlreadySet(attitude());

    // Then: 반영만 info이고 무시 두 경우는 warn이며, 경우별 문구와 자세값을 담는다.
    ASSERT_EQ(_messages.size(), 3u);
    EXPECT_EQ(_messages[0].level, spdlog::level::info);
    EXPECT_EQ(_messages[1].level, spdlog::level::warn);
    EXPECT_EQ(_messages[2].level, spdlog::level::warn);
    EXPECT_EQ(_messages[0].text, "Attitude applied: roll=1.5, pitch=-2.5, yaw=30");
    EXPECT_EQ(_messages[1].text, "Attitude ignored (operation state is OFF): roll=1.5, pitch=-2.5, yaw=30");
    EXPECT_EQ(_messages[2].text, "Attitude ignored (already set): roll=1.5, pitch=-2.5, yaw=30");
}

// 버려진 요청은 상태와 요청 ID를 담아 warn으로 기록한다.
TEST_F(BeamEventMessageLoggerTest, LogsRejectedRequestAsWarn)
{
    // When: 준비 전과 영역 밖 요청 거부를 기록한다.
    _beamLogger.logRequestBeamRejected(beam::RequestBeamResult(beam::RequestBeamStatus::NOT_READY, 11));
    _beamLogger.logRequestBeamRejected(beam::RequestBeamResult(beam::RequestBeamStatus::OUT_OF_COVERAGE, 12));

    // Then: warn으로 상태 이름과 요청 ID를 기록한다.
    ASSERT_EQ(_messages.size(), 2u);
    EXPECT_EQ(_messages[0].level, spdlog::level::warn);
    EXPECT_EQ(_messages[0].text, "Beam request rejected: status=NOT_READY, requestId=11");
    EXPECT_EQ(_messages[1].text, "Beam request rejected: status=OUT_OF_COVERAGE, requestId=12");
}

// 로거가 등록되지 않았으면 기록하지 않고 예외도 던지지 않는다.
TEST(BeamEventMessageLoggerWithoutRegistryTest, SkipsWhenLoggerIsNotRegistered)
{
    // Given: 등록된 로거가 없다.
    BeamEventMessageLogger beamLogger;
    ASSERT_EQ(spdlog::get(BeamEventMessageLogger::LOGGER_NAME), nullptr);

    // When / Then: 기록을 요청해도 예외 없이 지나간다.
    EXPECT_NO_THROW(beamLogger.logOperationStateChanged(beam::OperationState::ON));
}
}
}
