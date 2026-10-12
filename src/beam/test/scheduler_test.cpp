#include "scheduler/scheduler.hpp"

#include "attitude/attitude_error.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>

using namespace std::chrono_literals;
using namespace math::literals;

namespace
{

constexpr std::size_t AZIMUTH_COUNT = 21;

beam::RadarAttitude makeAttitude(math::Angle roll, math::Angle pitch, math::Angle yaw)
{
    beam::RadarAttitude attitude{};
    attitude.latitude = 37.0_deg;
    attitude.longitude = 127.0_deg;
    attitude.altitude = 0.1_km;
    attitude.roll = roll;
    attitude.pitch = pitch;
    attitude.yaw = yaw;
    return attitude;
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
    double azimuth_deg,
    double elevation_deg
) {
    return beam::BeamRequest::Builder()
        .beamType(beamType)
        .transmitTime(transmitTime)
        .requestId(0)
        .azimuth_ant(math::Angle::fromDegrees(azimuth_deg))
        .elevation_ant(math::Angle::fromDegrees(elevation_deg))
        .build();
}

// 영 자세에서 탐색 빔 index번째(0부터)의 방위각/고각
double gridAzimuthDeg(std::size_t index)
{
    return -42.0 + 4.2 * static_cast<double>(index % AZIMUTH_COUNT);
}

double gridElevationDeg(std::size_t index)
{
    return 3.0 + 4.4 * static_cast<double>(index / AZIMUTH_COUNT);
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

TEST(SchedulerTest, SecondAttitudeIsIgnored)
{
    beam::Scheduler scheduler;
    makeReady(scheduler, makeZeroAttitude());

    scheduler.setAttitude(makeAttitude(0_deg, 0_deg, 10_deg));
    const auto command = scheduler.next(0ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_NEAR(command->azimuth_ant.deg(), gridAzimuthDeg(0), 1e-4);
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
        EXPECT_NEAR(command->azimuth_ant.deg(), gridAzimuthDeg(index), 1e-4) << "index=" << index;
        EXPECT_NEAR(command->elevation_ant.deg(), gridElevationDeg(index), 1e-4) << "index=" << index;
        EXPECT_DOUBLE_EQ(command->azimuthBeamWidth.deg(), 6.0);
        EXPECT_DOUBLE_EQ(command->elevationBeamWidth.deg(), 6.0);
    }
}

TEST(SchedulerTest, SearchCursorWrapsAroundAfterLastBeam)
{
    beam::Scheduler scheduler;
    makeReady(scheduler);

    for (std::size_t index = 0; index < beam::BeamTable::size(); ++index)
    {
        ASSERT_TRUE(scheduler.next(std::chrono::milliseconds(10 * index)).has_value());
    }

    const auto wrapped = scheduler.next(std::chrono::milliseconds(10 * beam::BeamTable::size()));

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
    EXPECT_NEAR(command->azimuth_ant.deg(), gridAzimuthDeg(0) - 10.0, 1e-4);
    EXPECT_NEAR(command->elevation_ant.deg(), gridElevationDeg(0), 1e-4);
}

// ---------- 요청 빔 ----------

TEST(SchedulerTest, ConfirmationRequestProducesConfirmationBeam)
{
    beam::Scheduler scheduler;
    makeReady(scheduler);
    scheduler.addRequest(makeRequest(beam::BeamRequest::BeamType::CONFIRMATION, 100ms, 12.5, 20.5));

    const auto command = scheduler.next(100ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamCommand::BeamType::CONFIRMATION);
    EXPECT_EQ(command->beamId, 0u);
    EXPECT_EQ(command->transmitTime, 100ms);
    EXPECT_DOUBLE_EQ(command->azimuth_ant.deg(), 12.5);
    EXPECT_DOUBLE_EQ(command->elevation_ant.deg(), 20.5);
    EXPECT_DOUBLE_EQ(command->azimuthBeamWidth.deg(), 3.0);
    EXPECT_DOUBLE_EQ(command->elevationBeamWidth.deg(), 3.0);
}

TEST(SchedulerTest, TrackingRequestProducesTrackingBeam)
{
    beam::Scheduler scheduler;
    makeReady(scheduler);
    scheduler.addRequest(makeRequest(beam::BeamRequest::BeamType::TRACKING, 100ms, -30.0, 5.0));

    const auto command = scheduler.next(100ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamCommand::BeamType::TRACKING);
    EXPECT_EQ(command->beamId, 0u);
    EXPECT_DOUBLE_EQ(command->azimuth_ant.deg(), -30.0);
    EXPECT_DOUBLE_EQ(command->elevation_ant.deg(), 5.0);
    EXPECT_DOUBLE_EQ(command->azimuthBeamWidth.deg(), 3.0);
    EXPECT_DOUBLE_EQ(command->elevationBeamWidth.deg(), 3.0);
}

TEST(SchedulerTest, FutureRequestKeepsSearchingUntilItsTime)
{
    beam::Scheduler scheduler;
    makeReady(scheduler);
    scheduler.addRequest(makeRequest(beam::BeamRequest::BeamType::TRACKING, 30ms, 1.0, 2.0));

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

    scheduler.addRequest(makeRequest(beam::BeamRequest::BeamType::CONFIRMATION, 20ms, 1.0, 2.0));

    EXPECT_EQ(scheduler.next(20ms)->beamType, beam::BeamCommand::BeamType::CONFIRMATION);
    EXPECT_EQ(scheduler.next(30ms)->beamId, 3u);
}

TEST(SchedulerTest, DelayedRequestUsesCurrentTimeAsTransmitTime)
{
    beam::Scheduler scheduler;
    makeReady(scheduler);
    scheduler.addRequest(makeRequest(beam::BeamRequest::BeamType::TRACKING, 1005ms, 1.0, 2.0));

    const auto command = scheduler.next(1010ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamCommand::BeamType::TRACKING);
    EXPECT_EQ(command->transmitTime, 1010ms);
}

TEST(SchedulerTest, ExpiredRequestIsDroppedAndSearchBeamIsReturned)
{
    beam::Scheduler scheduler;
    makeReady(scheduler);
    scheduler.addRequest(makeRequest(beam::BeamRequest::BeamType::TRACKING, 1000ms, 1.0, 2.0));

    const auto command = scheduler.next(1030ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamCommand::BeamType::SEARCH);
    EXPECT_EQ(scheduler.next(1040ms)->beamType, beam::BeamCommand::BeamType::SEARCH);
}

TEST(SchedulerTest, CrowdedTicksFollowDesignedScenario)
{
    // 설계 문서의 예시: 1005 추적 A, 1010 확인 B, 1010 추적 C, 1010 추적 D
    beam::Scheduler scheduler;
    makeReady(scheduler);
    scheduler.addRequest(makeRequest(beam::BeamRequest::BeamType::TRACKING, 1005ms, 1.0, 0.0));
    scheduler.addRequest(makeRequest(beam::BeamRequest::BeamType::CONFIRMATION, 1010ms, 2.0, 0.0));
    scheduler.addRequest(makeRequest(beam::BeamRequest::BeamType::TRACKING, 1010ms, 3.0, 0.0));
    scheduler.addRequest(makeRequest(beam::BeamRequest::BeamType::TRACKING, 1010ms, 4.0, 0.0));

    const auto at1000 = scheduler.next(1000ms);
    EXPECT_EQ(at1000->beamType, beam::BeamCommand::BeamType::SEARCH);
    EXPECT_EQ(at1000->beamId, 1u);

    const auto at1010 = scheduler.next(1010ms);
    EXPECT_EQ(at1010->beamType, beam::BeamCommand::BeamType::TRACKING);
    EXPECT_DOUBLE_EQ(at1010->azimuth_ant.deg(), 1.0);

    const auto at1020 = scheduler.next(1020ms);
    EXPECT_EQ(at1020->beamType, beam::BeamCommand::BeamType::CONFIRMATION);
    EXPECT_DOUBLE_EQ(at1020->azimuth_ant.deg(), 2.0);

    const auto at1030 = scheduler.next(1030ms);
    EXPECT_EQ(at1030->beamType, beam::BeamCommand::BeamType::TRACKING);
    EXPECT_DOUBLE_EQ(at1030->azimuth_ant.deg(), 3.0);

    // D는 1010ms 요청이 30ms 지연되어 만료되므로 탐색 빔이 나간다.
    const auto at1040 = scheduler.next(1040ms);
    EXPECT_EQ(at1040->beamType, beam::BeamCommand::BeamType::SEARCH);
    EXPECT_EQ(at1040->beamId, 2u);
}

// ---------- 명령 카운트 ----------

TEST(SchedulerTest, CommandCountStartsAtOneAndIncreasesPerCommand)
{
    beam::Scheduler scheduler;
    makeReady(scheduler);

    EXPECT_EQ(scheduler.next(0ms)->commandCount, 1u);
    EXPECT_EQ(scheduler.next(10ms)->commandCount, 2u);

    scheduler.addRequest(makeRequest(beam::BeamRequest::BeamType::TRACKING, 20ms, 1.0, 2.0));

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
    scheduler.addRequest(makeRequest(beam::BeamRequest::BeamType::TRACKING, 0ms, 1.0, 2.0));
    makeReady(scheduler);

    EXPECT_EQ(scheduler.next(0ms)->beamType, beam::BeamCommand::BeamType::SEARCH);
}

TEST(SchedulerTest, RequestBeforeAttitudeIsIgnored)
{
    beam::Scheduler scheduler;
    scheduler.setOperationState(beam::OperationState::ON);
    scheduler.addRequest(makeRequest(beam::BeamRequest::BeamType::TRACKING, 0ms, 1.0, 2.0));
    scheduler.setAttitude(makeZeroAttitude());

    EXPECT_EQ(scheduler.next(0ms)->beamType, beam::BeamCommand::BeamType::SEARCH);
}

// ---------- 초기화 ----------

TEST(SchedulerTest, TurningOnAgainAfterOffResetsEverything)
{
    beam::Scheduler scheduler;
    makeReady(scheduler, makeAttitude(0_deg, 0_deg, 10_deg));
    scheduler.next(0ms);
    scheduler.next(10ms);
    scheduler.addRequest(makeRequest(beam::BeamRequest::BeamType::TRACKING, 1000ms, 1.0, 2.0));

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
    EXPECT_NEAR(first->azimuth_ant.deg(), gridAzimuthDeg(0), 1e-4);
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
