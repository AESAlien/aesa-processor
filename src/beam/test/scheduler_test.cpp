#include "scheduler/scheduler.hpp"

#include "attitude/attitude_error.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>

using namespace std::chrono_literals;
using namespace math::literals;

namespace
{

constexpr std::size_t AZIMUTH_COUNT = 21;

beam::RadarAttitude makeAttitude(math::Angle roll, math::Angle pitch, math::Angle yaw)
{
    return beam::RadarAttitude(37.0_deg, 127.0_deg, 0.1_km, roll, pitch, yaw);
}

beam::RadarAttitude makeZeroAttitude()
{
    return makeAttitude(0_deg, 0_deg, 0_deg);
}

// ON으로 바꾸고 자세정보까지 넣어 준비 상태로 만든다.
void makeReady(beam::Scheduler& scheduler, const beam::RadarAttitude& attitude)
{
    scheduler.setOperationState(beam::OperationState::ON);
    scheduler.setAttitude(attitude);
}

void makeReady(beam::Scheduler& scheduler)
{
    makeReady(scheduler, makeZeroAttitude());
}

beam::BeamRequest makeRequest(
    beam::BeamRequest::BeamType beamType,
    std::chrono::milliseconds transmitTime,
    uint32_t requestId,
    math::Angle azimuth,
    math::Angle elevation
) {
    return beam::BeamRequest::Builder()
        .beamType(beamType)
        .transmitTime(transmitTime)
        .requestId(requestId)
        .azimuth_ant(azimuth)
        .elevation_ant(elevation)
        .build();
}

// 영 자세에서 탐색 빔 index번째(0부터)의 방위각/고각
math::Angle gridAzimuth(std::size_t index)
{
    return -42_deg + 4.2_deg * static_cast<double>(index % AZIMUTH_COUNT);
}

math::Angle gridElevation(std::size_t index)
{
    return 3_deg + 4.4_deg * static_cast<double>(index / AZIMUTH_COUNT);
}

} // namespace

// ---------- 준비 상태 ----------

TEST(SchedulerTest, ReturnsNothingInitially)
{
    beam::Scheduler scheduler;

    EXPECT_FALSE(scheduler.next(0ms).has_value());
}

TEST(SchedulerTest, ReturnsNothingWhenOnButAttitudeNotReceived)
{
    beam::Scheduler scheduler;
    scheduler.setOperationState(beam::OperationState::ON);

    EXPECT_FALSE(scheduler.next(0ms).has_value());
}

TEST(SchedulerTest, AttitudeReceivedWhileOffIsIgnored)
{
    beam::Scheduler scheduler;
    scheduler.setAttitude(makeZeroAttitude());
    scheduler.setOperationState(beam::OperationState::ON);

    EXPECT_FALSE(scheduler.next(0ms).has_value());
}

TEST(SchedulerTest, AttitudeAfterOnMakesSchedulerReady)
{
    beam::Scheduler scheduler;
    makeReady(scheduler);

    EXPECT_TRUE(scheduler.next(0ms).has_value());
}

TEST(SchedulerTest, ReturnsNothingAfterTurnedOff)
{
    beam::Scheduler scheduler;
    makeReady(scheduler);
    ASSERT_TRUE(scheduler.next(0ms).has_value());

    scheduler.setOperationState(beam::OperationState::OFF);

    EXPECT_FALSE(scheduler.next(10ms).has_value());
}

TEST(SchedulerTest, InvalidAttitudeThrowsAndLeavesSchedulerNotReady)
{
    beam::Scheduler scheduler;
    scheduler.setOperationState(beam::OperationState::ON);

    EXPECT_THROW(scheduler.setAttitude(makeAttitude(0_deg, 90_deg, 0_deg)), beam::AttitudeError);
    EXPECT_FALSE(scheduler.next(0ms).has_value());

    // 이후 올바른 자세정보를 받으면 정상 동작한다.
    scheduler.setAttitude(makeZeroAttitude());
    EXPECT_TRUE(scheduler.next(10ms).has_value());
}

TEST(SchedulerTest, GimbalLockAttitudeThrowsEvenWhileOff)
{
    beam::Scheduler scheduler;

    try
    {
        scheduler.setAttitude(makeAttitude(0_deg, 90_deg, 0_deg));
        ADD_FAILURE() << "AttitudeError not thrown";
    }
    catch (const beam::AttitudeError& error)
    {
        EXPECT_EQ(error.code(), beam::AttitudeErrorCode::GIMBAL_LOCK);
    }

    // 예외 이후에도 상태가 변하지 않아, ON 뒤 올바른 자세정보를 받으면 정상 동작한다.
    scheduler.setOperationState(beam::OperationState::ON);
    EXPECT_FALSE(scheduler.next(0ms).has_value());
    scheduler.setAttitude(makeZeroAttitude());
    EXPECT_TRUE(scheduler.next(10ms).has_value());
}

TEST(SchedulerTest, GimbalLockAttitudeThrowsAfterSearchBeamTableIsCreated)
{
    beam::Scheduler scheduler;
    makeReady(scheduler, makeZeroAttitude());
    ASSERT_TRUE(scheduler.next(0ms).has_value());

    EXPECT_THROW(scheduler.setAttitude(makeAttitude(0_deg, 90_deg, 0_deg)), beam::AttitudeError);

    // 기존 SearchBeamTable과 탐색 순서는 그대로 유지된다.
    const auto command = scheduler.next(10ms);
    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamId, 2u);
    EXPECT_EQ(command->azimuth_ant, gridAzimuth(1));
}

TEST(SchedulerTest, GimbalLockUsesPitchLimitFromConfig)
{
    const beam::BeamConfig config = beam::BeamConfig::Builder()
        .attitudeConfig(beam::AttitudeConfig::Builder().maxAbsPitch(60_deg).build())
        .build();
    beam::Scheduler scheduler(config);

    EXPECT_THROW(scheduler.setAttitude(makeAttitude(0_deg, 60_deg, 0_deg)), beam::AttitudeError);
    EXPECT_THROW(scheduler.setAttitude(makeAttitude(0_deg, -70_deg, 0_deg)), beam::AttitudeError);
    EXPECT_NO_THROW(scheduler.setAttitude(makeAttitude(0_deg, 59_deg, 0_deg)));
}

TEST(SchedulerTest, SecondAttitudeIsIgnored)
{
    beam::Scheduler scheduler;
    makeReady(scheduler, makeZeroAttitude());

    scheduler.setAttitude(makeAttitude(0_deg, 0_deg, 10_deg));
    const auto command = scheduler.next(0ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->azimuth_ant, gridAzimuth(0));
}

// ---------- 탐색 빔 ----------

TEST(SchedulerTest, ReturnsSearchBeamsInGridOrderWhenNoRequests)
{
    beam::Scheduler scheduler;
    makeReady(scheduler);

    for (std::size_t index = 0; index < 30; ++index)
    {
        const auto command = scheduler.next(std::chrono::milliseconds(10 * index));

        ASSERT_TRUE(command.has_value()) << "index=" << index;
        EXPECT_EQ(command->beamType, beam::BeamCommand::BeamType::SEARCH) << "index=" << index;
        EXPECT_EQ(command->beamId, static_cast<std::uint32_t>(index + 1)) << "index=" << index;
        EXPECT_EQ(command->azimuth_ant, gridAzimuth(index)) << "index=" << index;
        EXPECT_EQ(command->elevation_ant, gridElevation(index)) << "index=" << index;
        EXPECT_EQ(command->azimuthBeamWidth, 6_deg);
        EXPECT_EQ(command->elevationBeamWidth, 6_deg);
    }
}

TEST(SchedulerTest, SearchCursorWrapsAroundAfterLastBeam)
{
    beam::Scheduler scheduler;
    makeReady(scheduler);

    beam::SearchBeamTable table(beam::AntEnuTransform(makeZeroAttitude(), beam::AttitudeConfig{}), beam::SearchGridConfig{});
    for (std::size_t index = 0; index < table.size(); ++index)
    {
        ASSERT_TRUE(scheduler.next(std::chrono::milliseconds(10 * index)).has_value());
    }

    const auto wrapped = scheduler.next(std::chrono::milliseconds(10 * table.size()));

    ASSERT_TRUE(wrapped.has_value());
    EXPECT_EQ(wrapped->beamId, 1u);
}

TEST(SchedulerTest, SearchBeamTransmitTimeIsCurrentTime)
{
    beam::Scheduler scheduler;
    makeReady(scheduler);

    EXPECT_EQ(scheduler.next(1230ms)->transmitTime, 1230ms);
    EXPECT_EQ(scheduler.next(1240ms)->transmitTime, 1240ms);
}

TEST(SchedulerTest, SearchBeamsFollowAttitude)
{
    // 안테나가 yaw 만큼 돌아가 있으면 고정 그리드는 안테나 기준으로 -yaw 만큼 이동한다.
    beam::Scheduler scheduler;
    makeReady(scheduler, makeAttitude(0_deg, 0_deg, 10_deg));

    const auto command = scheduler.next(0ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->azimuth_ant, gridAzimuth(0) - 10_deg);
    EXPECT_EQ(command->elevation_ant, gridElevation(0));
}

// ---------- 요청 빔 ----------

TEST(SchedulerTest, ConfirmationRequestProducesConfirmationBeam)
{
    beam::Scheduler scheduler;
    makeReady(scheduler);
    scheduler.addRequest(makeRequest(beam::BeamRequest::BeamType::CONFIRMATION, 100ms, 20, 12.5_deg, 20.5_deg));

    const auto command = scheduler.next(100ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamCommand::BeamType::CONFIRMATION);
    EXPECT_EQ(command->beamId, 20u);
    EXPECT_EQ(command->transmitTime, 100ms);
    EXPECT_EQ(command->azimuth_ant, 12.5_deg);
    EXPECT_EQ(command->elevation_ant, 20.5_deg);
    EXPECT_EQ(command->azimuthBeamWidth, 3_deg);
    EXPECT_EQ(command->elevationBeamWidth, 3_deg);
}

TEST(SchedulerTest, TrackingRequestProducesTrackingBeam)
{
    beam::Scheduler scheduler;
    makeReady(scheduler);
    scheduler.addRequest(makeRequest(beam::BeamRequest::BeamType::TRACKING, 100ms, 100, -30_deg, 5_deg));

    const auto command = scheduler.next(100ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamCommand::BeamType::TRACKING);
    EXPECT_EQ(command->beamId, 100u);
    EXPECT_EQ(command->azimuth_ant, -30_deg);
    EXPECT_EQ(command->elevation_ant, 5_deg);
    EXPECT_EQ(command->azimuthBeamWidth, 3_deg);
    EXPECT_EQ(command->elevationBeamWidth, 3_deg);
}

TEST(SchedulerTest, FutureRequestKeepsSearchingUntilItsTime)
{
    beam::Scheduler scheduler;
    makeReady(scheduler);
    scheduler.addRequest(makeRequest(beam::BeamRequest::BeamType::TRACKING, 30ms, 20, 1_deg, 2_deg));

    EXPECT_EQ(scheduler.next(0ms)->beamType, beam::BeamCommand::BeamType::SEARCH);
    EXPECT_EQ(scheduler.next(10ms)->beamType, beam::BeamCommand::BeamType::SEARCH);
    EXPECT_EQ(scheduler.next(20ms)->beamType, beam::BeamCommand::BeamType::SEARCH);
    EXPECT_EQ(scheduler.next(30ms)->beamType, beam::BeamCommand::BeamType::TRACKING);
}

TEST(SchedulerTest, RequestBeamDoesNotAdvanceSearchCursor)
{
    beam::Scheduler scheduler;
    makeReady(scheduler);

    EXPECT_EQ(scheduler.next(0ms)->beamId, 1u);
    EXPECT_EQ(scheduler.next(10ms)->beamId, 2u);

    scheduler.addRequest(makeRequest(beam::BeamRequest::BeamType::CONFIRMATION, 20ms, 10, 1_deg, 2_deg));

    EXPECT_EQ(scheduler.next(20ms)->beamType, beam::BeamCommand::BeamType::CONFIRMATION);
    EXPECT_EQ(scheduler.next(30ms)->beamId, 3u);
}

TEST(SchedulerTest, DelayedRequestUsesCurrentTimeAsTransmitTime)
{
    beam::Scheduler scheduler;
    makeReady(scheduler);
    scheduler.addRequest(makeRequest(beam::BeamRequest::BeamType::TRACKING, 1005ms, 10, 1_deg, 2_deg));

    const auto command = scheduler.next(1010ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamCommand::BeamType::TRACKING);
    EXPECT_EQ(command->transmitTime, 1010ms);
}

TEST(SchedulerTest, VeryOldRequestIsStillSentAsRequestBeam)
{
    // 요청은 만료로 폐기되지 않는다. 오래 기다렸어도 현재 시각의 요청 빔으로 나가고 탐색 순서는 밀리지 않는다.
    beam::Scheduler scheduler;
    makeReady(scheduler);
    scheduler.addRequest(makeRequest(beam::BeamRequest::BeamType::TRACKING, 1000ms, 7, 1_deg, 2_deg));

    const auto command = scheduler.next(10000ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamCommand::BeamType::TRACKING);
    EXPECT_EQ(command->beamId, 7u);
    EXPECT_EQ(command->transmitTime, 10000ms);

    const auto next = scheduler.next(10010ms);
    ASSERT_TRUE(next.has_value());
    EXPECT_EQ(next->beamType, beam::BeamCommand::BeamType::SEARCH);
    EXPECT_EQ(next->beamId, 1u);
}

TEST(SchedulerTest, CrowdedTicksFollowDesignedScenario)
{
    // 1005 추적 A, 1010 확인 B, 1010 추적 C, 1010 추적 D
    beam::Scheduler scheduler;
    makeReady(scheduler);
    scheduler.addRequest(makeRequest(beam::BeamRequest::BeamType::TRACKING, 1005ms, 1, 1_deg, 0_deg));
    scheduler.addRequest(makeRequest(beam::BeamRequest::BeamType::CONFIRMATION, 1010ms, 2, 2_deg, 0_deg));
    scheduler.addRequest(makeRequest(beam::BeamRequest::BeamType::TRACKING, 1010ms, 3, 3_deg, 0_deg));
    scheduler.addRequest(makeRequest(beam::BeamRequest::BeamType::TRACKING, 1010ms, 4, 4_deg, 0_deg));

    const auto at1000 = scheduler.next(1000ms);
    EXPECT_EQ(at1000->beamType, beam::BeamCommand::BeamType::SEARCH);
    EXPECT_EQ(at1000->beamId, 1u);

    const auto at1010 = scheduler.next(1010ms);
    EXPECT_EQ(at1010->beamType, beam::BeamCommand::BeamType::TRACKING);
    EXPECT_EQ(at1010->azimuth_ant, 1_deg);

    const auto at1020 = scheduler.next(1020ms);
    EXPECT_EQ(at1020->beamType, beam::BeamCommand::BeamType::CONFIRMATION);
    EXPECT_EQ(at1020->azimuth_ant, 2_deg);

    const auto at1030 = scheduler.next(1030ms);
    EXPECT_EQ(at1030->beamType, beam::BeamCommand::BeamType::TRACKING);
    EXPECT_EQ(at1030->azimuth_ant, 3_deg);

    // D는 1010ms 요청이 30ms 지연되어 1040ms에 송신된다. 폐기되지 않는다.
    const auto at1040 = scheduler.next(1040ms);
    EXPECT_EQ(at1040->beamType, beam::BeamCommand::BeamType::TRACKING);
    EXPECT_EQ(at1040->azimuth_ant, 4_deg);

    const auto at1050 = scheduler.next(1050ms);
    EXPECT_EQ(at1050->beamType, beam::BeamCommand::BeamType::SEARCH);
    EXPECT_EQ(at1050->beamId, 2u);
}

// ---------- 명령 카운트 ----------

TEST(SchedulerTest, CommandCountStartsAtOneAndIncreasesPerCommand)
{
    beam::Scheduler scheduler;
    makeReady(scheduler);

    EXPECT_EQ(scheduler.next(0ms)->commandCount, 1u);
    EXPECT_EQ(scheduler.next(10ms)->commandCount, 2u);

    scheduler.addRequest(makeRequest(beam::BeamRequest::BeamType::TRACKING, 20ms, 100, 1_deg, 2_deg));

    EXPECT_EQ(scheduler.next(20ms)->commandCount, 3u);
    EXPECT_EQ(scheduler.next(30ms)->commandCount, 4u);
}

TEST(SchedulerTest, CommandCountDoesNotIncreaseWhenNothingIsSent)
{
    beam::Scheduler scheduler;
    scheduler.setOperationState(beam::OperationState::ON);

    EXPECT_FALSE(scheduler.next(0ms).has_value());

    scheduler.setAttitude(makeZeroAttitude());

    EXPECT_EQ(scheduler.next(10ms)->commandCount, 1u);
}

// ---------- 요청 접수 조건 ----------

TEST(SchedulerTest, RequestWhileOffIsIgnored)
{
    beam::Scheduler scheduler;
    scheduler.addRequest(makeRequest(beam::BeamRequest::BeamType::TRACKING, 0ms, 3, 1_deg, 2_deg));
    makeReady(scheduler);

    EXPECT_EQ(scheduler.next(0ms)->beamType, beam::BeamCommand::BeamType::SEARCH);
}

TEST(SchedulerTest, RequestBeforeAttitudeIsIgnored)
{
    beam::Scheduler scheduler;
    scheduler.setOperationState(beam::OperationState::ON);
    scheduler.addRequest(makeRequest(beam::BeamRequest::BeamType::TRACKING, 0ms, 4, 1_deg, 2_deg));
    scheduler.setAttitude(makeZeroAttitude());

    EXPECT_EQ(scheduler.next(0ms)->beamType, beam::BeamCommand::BeamType::SEARCH);
}

// ---------- 알 수 없는 요청 종류 ----------

namespace
{

beam::BeamRequest makeUnknownTypeRequest()
{
    return makeRequest(static_cast<beam::BeamRequest::BeamType>(0), 0ms, 9, 1_deg, 2_deg);
}

} // namespace

TEST(SchedulerTest, UnknownRequestTypeThrowsAndIsNotQueued)
{
    beam::Scheduler scheduler;
    makeReady(scheduler);

    EXPECT_THROW(scheduler.addRequest(makeUnknownTypeRequest()), std::invalid_argument);

    // 거부된 요청은 큐에 남지 않으므로 다음 틱은 정상적으로 탐색 빔이 나간다.
    const auto command = scheduler.next(0ms);
    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamCommand::BeamType::SEARCH);
    EXPECT_EQ(command->beamId, 1u);
}

TEST(SchedulerTest, UnknownRequestTypeIsIgnoredWhileOffOrBeforeAttitude)
{
    beam::Scheduler scheduler;
    EXPECT_NO_THROW(scheduler.addRequest(makeUnknownTypeRequest()));

    scheduler.setOperationState(beam::OperationState::ON);
    EXPECT_NO_THROW(scheduler.addRequest(makeUnknownTypeRequest()));
}

// ---------- 초기화 ----------

TEST(SchedulerTest, TurningOnAgainAfterOffResetsEverything)
{
    beam::Scheduler scheduler;
    makeReady(scheduler, makeAttitude(0_deg, 0_deg, 10_deg));
    scheduler.next(0ms);
    scheduler.next(10ms);
    scheduler.addRequest(makeRequest(beam::BeamRequest::BeamType::TRACKING, 1000ms, 6, 1_deg, 2_deg));

    scheduler.setOperationState(beam::OperationState::OFF);
    scheduler.setOperationState(beam::OperationState::ON);

    // 자세정보를 다시 받기 전에는 준비 상태가 아니다.
    EXPECT_FALSE(scheduler.next(20ms).has_value());

    // 새 운용에서는 새 자세(영 자세)로 만든 테이블, 처음 구역, 카운트 1부터 시작한다.
    scheduler.setAttitude(makeZeroAttitude());
    const auto first = scheduler.next(1000ms);

    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(first->beamType, beam::BeamCommand::BeamType::SEARCH);
    EXPECT_EQ(first->beamId, 1u);
    EXPECT_EQ(first->commandCount, 1u);
    EXPECT_EQ(first->azimuth_ant, gridAzimuth(0));
}

TEST(SchedulerTest, TurningOnWhileAlreadyOnKeepsState)
{
    beam::Scheduler scheduler;
    makeReady(scheduler);
    scheduler.next(0ms);

    scheduler.setOperationState(beam::OperationState::ON);

    const auto command = scheduler.next(10ms);
    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamId, 2u);
    EXPECT_EQ(command->commandCount, 2u);
}

TEST(SchedulerTest, TurningOffWhileAlreadyOffDoesNothing)
{
    beam::Scheduler scheduler;
    scheduler.setOperationState(beam::OperationState::OFF);
    makeReady(scheduler);

    EXPECT_TRUE(scheduler.next(0ms).has_value());
}

// ---------- 요청 빔 설정 반영 ----------
// 설정 값 검증(빔폭 0 이하, NaN, Inf)은 config_test.cpp 에서 확인한다.

namespace
{

beam::BeamConfig makeConfigWithRequestBeamWidth(math::Angle confirmationWidth, math::Angle trackingWidth)
{
    return beam::BeamConfig::Builder()
        .requestBeamConfig(beam::RequestBeamConfig::Builder()
            .confirmationBeamWidth(confirmationWidth)
            .trackingBeamWidth(trackingWidth)
            .build())
        .build();
}

} // namespace

TEST(SchedulerTest, ConfiguredRequestBeamWidthIsUsedPerBeamType)
{
    beam::Scheduler scheduler(makeConfigWithRequestBeamWidth(2_deg, 4_deg));
    makeReady(scheduler);

    scheduler.addRequest(makeRequest(beam::BeamRequest::BeamType::CONFIRMATION, 0ms, 1, 1_deg, 5_deg));
    const auto confirmation = scheduler.next(0ms);
    ASSERT_TRUE(confirmation.has_value());
    EXPECT_EQ(confirmation->beamType, beam::BeamCommand::BeamType::CONFIRMATION);
    EXPECT_EQ(confirmation->azimuthBeamWidth, 2_deg);
    EXPECT_EQ(confirmation->elevationBeamWidth, 2_deg);

    scheduler.addRequest(makeRequest(beam::BeamRequest::BeamType::TRACKING, 10ms, 2, 1_deg, 5_deg));
    const auto tracking = scheduler.next(10ms);
    ASSERT_TRUE(tracking.has_value());
    EXPECT_EQ(tracking->beamType, beam::BeamCommand::BeamType::TRACKING);
    EXPECT_EQ(tracking->azimuthBeamWidth, 4_deg);
    EXPECT_EQ(tracking->elevationBeamWidth, 4_deg);
}
