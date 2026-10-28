#include "scheduler/scheduler.hpp"
#include "scheduler/beam_request_factory.hpp"

#include "attitude/ant_enu_transform.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>

using namespace std::chrono_literals;
using namespace math::literals;

namespace
{

constexpr std::size_t AZIMUTH_COUNT = 21;

const beam::SearchGridConfig GRID_CONFIG(
    AZIMUTH_COUNT,
    9,
    math::Azimuth(-42_deg),
    4.2_deg,
    math::Elevation(3_deg),
    4.4_deg,
    beam::BeamWidth(math::AngularWidth(6_deg), math::AngularWidth(6_deg)));

// 자세정보로 탐색 격자를 만드는 일은 BeamController가 맡으므로, 테스트에서는 격자를 직접 만들어 넘긴다.
beam::SearchGrid makeSearchGrid(math::Angle roll, math::Angle pitch, math::Angle yaw)
{
    const beam::AntEnuTransform transform(
        beam::RadarAttitude(roll, pitch, yaw),
        beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}));
    return beam::SearchGrid(transform, GRID_CONFIG);
}

beam::SearchGrid makeZeroSearchGrid()
{
    return makeSearchGrid(0_deg, 0_deg, 0_deg);
}

// ON으로 바꾸고 탐색 격자까지 넣어 준비 상태로 만든다.
void makeReady(beam::Scheduler& scheduler, beam::SearchGrid searchGrid)
{
    scheduler.setOperationState(beam::OperationState::ON);
    scheduler.setSearchGrid(std::move(searchGrid));
}

void makeReady(beam::Scheduler& scheduler)
{
    makeReady(scheduler, makeZeroSearchGrid());
}

// 요청의 빔 폭과 우선순위는 팩토리가 정한다. 설정을 지정하지 않으면 기본 설정의 팩토리를 쓴다.
beam::BeamRequest makeRequest(
    const beam::BeamRequestFactory& factory,
    beam::RequestBeamType beamType,
    std::chrono::milliseconds transmitTime,
    uint32_t requestId,
    math::Angle azimuth,
    math::Angle elevation
) {
    return factory.create(beam::RequestBeamCommand(
        beamType, transmitTime, requestId, math::SphericalDirection(math::Azimuth(azimuth), math::Elevation(elevation))));
}

beam::BeamRequest makeRequest(
    beam::RequestBeamType beamType,
    std::chrono::milliseconds transmitTime,
    uint32_t requestId,
    math::Angle azimuth,
    math::Angle elevation
) {
    static const beam::BeamRequestFactory factory{
        beam::RequestBeamConfig(beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)), beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)))};
    return makeRequest(factory, beamType, transmitTime, requestId, azimuth, elevation);
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

TEST(SchedulerTest, ReturnsNothingWhenOnButSearchGridNotReceived)
{
    beam::Scheduler scheduler;
    scheduler.setOperationState(beam::OperationState::ON);

    EXPECT_FALSE(scheduler.next(0ms).has_value());
}

TEST(SchedulerTest, SearchGridReceivedWhileOffIsIgnored)
{
    beam::Scheduler scheduler;
    scheduler.setSearchGrid(makeZeroSearchGrid());
    scheduler.setOperationState(beam::OperationState::ON);

    EXPECT_FALSE(scheduler.next(0ms).has_value());
}

TEST(SchedulerTest, SearchGridAfterOnMakesSchedulerReady)
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

TEST(SchedulerTest, SecondSearchGridIsIgnored)
{
    beam::Scheduler scheduler;
    makeReady(scheduler, makeZeroSearchGrid());

    scheduler.setSearchGrid(makeSearchGrid(0_deg, 0_deg, 10_deg));
    const auto command = scheduler.next(0ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_TRUE(command->direction_ant.azimuth.isEquivalentTo(gridAzimuth(0)));
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
        EXPECT_EQ(command->beamType, beam::BeamType::SEARCH) << "index=" << index;
        EXPECT_EQ(command->beamId, static_cast<std::uint32_t>(index + 1)) << "index=" << index;
        EXPECT_TRUE(command->direction_ant.azimuth.isEquivalentTo(gridAzimuth(index))) << "index=" << index;
        EXPECT_EQ(command->direction_ant.elevation, gridElevation(index)) << "index=" << index;
        EXPECT_EQ(command->beamWidth.azimuth, 6_deg);
        EXPECT_EQ(command->beamWidth.elevation, 6_deg);
    }
}

TEST(SchedulerTest, SearchCursorWrapsAroundAfterLastBeam)
{
    beam::Scheduler scheduler;
    makeReady(scheduler);

    const std::size_t beamCount = GRID_CONFIG.azimuthCount() * GRID_CONFIG.elevationCount();
    for (std::size_t index = 0; index < beamCount; ++index)
    {
        ASSERT_TRUE(scheduler.next(std::chrono::milliseconds(10 * index)).has_value());
    }

    const auto wrapped = scheduler.next(std::chrono::milliseconds(10 * beamCount));

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

TEST(SchedulerTest, SearchBeamsFollowGivenSearchGrid)
{
    // 격자는 안테나 정면 기준이므로 안테나가 yaw 만큼 돌아가 있으면 ENU 방위각도 +yaw 만큼 이동한다.
    beam::Scheduler scheduler;
    makeReady(scheduler, makeSearchGrid(0_deg, 0_deg, 10_deg));

    const auto command = scheduler.next(0ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_TRUE(command->direction_ant.azimuth.isEquivalentTo(gridAzimuth(0) + 10_deg));
    EXPECT_EQ(command->direction_ant.elevation, gridElevation(0));
}

// ---------- 요청 빔 ----------

TEST(SchedulerTest, ConfirmationRequestProducesConfirmationBeam)
{
    beam::Scheduler scheduler;
    makeReady(scheduler);
    scheduler.addRequest(makeRequest(beam::RequestBeamType::CONFIRMATION, 100ms, 20, 12.5_deg, 20.5_deg));

    const auto command = scheduler.next(100ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamType::CONFIRMATION);
    EXPECT_EQ(command->beamId, 20u);
    EXPECT_EQ(command->transmitTime, 100ms);
    EXPECT_TRUE(command->direction_ant.azimuth.isEquivalentTo(12.5_deg));
    EXPECT_EQ(command->direction_ant.elevation, 20.5_deg);
    EXPECT_EQ(command->beamWidth.azimuth, 3_deg);
    EXPECT_EQ(command->beamWidth.elevation, 3_deg);
}

TEST(SchedulerTest, TrackingRequestProducesTrackingBeam)
{
    beam::Scheduler scheduler;
    makeReady(scheduler);
    scheduler.addRequest(makeRequest(beam::RequestBeamType::TRACKING, 100ms, 100, -30_deg, 5_deg));

    const auto command = scheduler.next(100ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamType::TRACKING);
    EXPECT_EQ(command->beamId, 100u);
    EXPECT_TRUE(command->direction_ant.azimuth.isEquivalentTo(-30_deg));
    EXPECT_EQ(command->direction_ant.elevation, 5_deg);
    EXPECT_EQ(command->beamWidth.azimuth, 3_deg);
    EXPECT_EQ(command->beamWidth.elevation, 3_deg);
}

TEST(SchedulerTest, FutureRequestKeepsSearchingUntilItsTime)
{
    beam::Scheduler scheduler;
    makeReady(scheduler);
    scheduler.addRequest(makeRequest(beam::RequestBeamType::TRACKING, 30ms, 20, 1_deg, 2_deg));

    EXPECT_EQ(scheduler.next(0ms)->beamType, beam::BeamType::SEARCH);
    EXPECT_EQ(scheduler.next(10ms)->beamType, beam::BeamType::SEARCH);
    EXPECT_EQ(scheduler.next(20ms)->beamType, beam::BeamType::SEARCH);
    EXPECT_EQ(scheduler.next(30ms)->beamType, beam::BeamType::TRACKING);
}

TEST(SchedulerTest, RequestBeamDoesNotAdvanceSearchCursor)
{
    beam::Scheduler scheduler;
    makeReady(scheduler);

    EXPECT_EQ(scheduler.next(0ms)->beamId, 1u);
    EXPECT_EQ(scheduler.next(10ms)->beamId, 2u);

    scheduler.addRequest(makeRequest(beam::RequestBeamType::CONFIRMATION, 20ms, 10, 1_deg, 2_deg));

    EXPECT_EQ(scheduler.next(20ms)->beamType, beam::BeamType::CONFIRMATION);
    EXPECT_EQ(scheduler.next(30ms)->beamId, 3u);
}

TEST(SchedulerTest, DelayedRequestUsesCurrentTimeAsTransmitTime)
{
    beam::Scheduler scheduler;
    makeReady(scheduler);
    scheduler.addRequest(makeRequest(beam::RequestBeamType::TRACKING, 1005ms, 10, 1_deg, 2_deg));

    const auto command = scheduler.next(1010ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamType::TRACKING);
    EXPECT_EQ(command->transmitTime, 1010ms);
}

TEST(SchedulerTest, VeryOldRequestIsStillSentAsRequestBeam)
{
    // 요청은 만료로 폐기되지 않는다. 오래 기다렸어도 현재 시각의 요청 빔으로 나가고 탐색 순서는 밀리지 않는다.
    beam::Scheduler scheduler;
    makeReady(scheduler);
    scheduler.addRequest(makeRequest(beam::RequestBeamType::TRACKING, 1000ms, 7, 1_deg, 2_deg));

    const auto command = scheduler.next(10000ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamType::TRACKING);
    EXPECT_EQ(command->beamId, 7u);
    EXPECT_EQ(command->transmitTime, 10000ms);

    const auto next = scheduler.next(10010ms);
    ASSERT_TRUE(next.has_value());
    EXPECT_EQ(next->beamType, beam::BeamType::SEARCH);
    EXPECT_EQ(next->beamId, 1u);
}

TEST(SchedulerTest, CrowdedTicksFollowDesignedScenario)
{
    // 1005 추적 A, 1010 확인 B, 1010 추적 C, 1010 추적 D
    beam::Scheduler scheduler;
    makeReady(scheduler);
    scheduler.addRequest(makeRequest(beam::RequestBeamType::TRACKING, 1005ms, 1, 1_deg, 0_deg));
    scheduler.addRequest(makeRequest(beam::RequestBeamType::CONFIRMATION, 1010ms, 2, 2_deg, 0_deg));
    scheduler.addRequest(makeRequest(beam::RequestBeamType::TRACKING, 1010ms, 3, 3_deg, 0_deg));
    scheduler.addRequest(makeRequest(beam::RequestBeamType::TRACKING, 1010ms, 4, 4_deg, 0_deg));

    const auto at1000 = scheduler.next(1000ms);
    EXPECT_EQ(at1000->beamType, beam::BeamType::SEARCH);
    EXPECT_EQ(at1000->beamId, 1u);

    const auto at1010 = scheduler.next(1010ms);
    EXPECT_EQ(at1010->beamType, beam::BeamType::TRACKING);
    EXPECT_TRUE(at1010->direction_ant.azimuth.isEquivalentTo(1_deg));

    const auto at1020 = scheduler.next(1020ms);
    EXPECT_EQ(at1020->beamType, beam::BeamType::CONFIRMATION);
    EXPECT_TRUE(at1020->direction_ant.azimuth.isEquivalentTo(2_deg));

    const auto at1030 = scheduler.next(1030ms);
    EXPECT_EQ(at1030->beamType, beam::BeamType::TRACKING);
    EXPECT_TRUE(at1030->direction_ant.azimuth.isEquivalentTo(3_deg));

    // D는 1010ms 요청이 30ms 지연되어 1040ms에 송신된다. 폐기되지 않는다.
    const auto at1040 = scheduler.next(1040ms);
    EXPECT_EQ(at1040->beamType, beam::BeamType::TRACKING);
    EXPECT_TRUE(at1040->direction_ant.azimuth.isEquivalentTo(4_deg));

    const auto at1050 = scheduler.next(1050ms);
    EXPECT_EQ(at1050->beamType, beam::BeamType::SEARCH);
    EXPECT_EQ(at1050->beamId, 2u);
}

// ---------- 명령 카운트 ----------

TEST(SchedulerTest, CommandCountStartsAtOneAndIncreasesPerCommand)
{
    beam::Scheduler scheduler;
    makeReady(scheduler);

    EXPECT_EQ(scheduler.next(0ms)->commandCount, 1u);
    EXPECT_EQ(scheduler.next(10ms)->commandCount, 2u);

    scheduler.addRequest(makeRequest(beam::RequestBeamType::TRACKING, 20ms, 100, 1_deg, 2_deg));

    EXPECT_EQ(scheduler.next(20ms)->commandCount, 3u);
    EXPECT_EQ(scheduler.next(30ms)->commandCount, 4u);
}

TEST(SchedulerTest, CommandCountDoesNotIncreaseWhenNothingIsSent)
{
    beam::Scheduler scheduler;
    scheduler.setOperationState(beam::OperationState::ON);

    EXPECT_FALSE(scheduler.next(0ms).has_value());

    scheduler.setSearchGrid(makeZeroSearchGrid());

    EXPECT_EQ(scheduler.next(10ms)->commandCount, 1u);
}

// ---------- 요청 접수 조건 ----------

TEST(SchedulerTest, RequestWhileOffIsIgnored)
{
    beam::Scheduler scheduler;
    scheduler.addRequest(makeRequest(beam::RequestBeamType::TRACKING, 0ms, 3, 1_deg, 2_deg));
    makeReady(scheduler);

    EXPECT_EQ(scheduler.next(0ms)->beamType, beam::BeamType::SEARCH);
}

TEST(SchedulerTest, RequestBeforeSearchGridIsIgnored)
{
    beam::Scheduler scheduler;
    scheduler.setOperationState(beam::OperationState::ON);
    scheduler.addRequest(makeRequest(beam::RequestBeamType::TRACKING, 0ms, 4, 1_deg, 2_deg));
    scheduler.setSearchGrid(makeZeroSearchGrid());

    EXPECT_EQ(scheduler.next(0ms)->beamType, beam::BeamType::SEARCH);
}

// ---------- 초기화 ----------

TEST(SchedulerTest, TurningOnAgainAfterOffResetsEverything)
{
    beam::Scheduler scheduler;
    makeReady(scheduler, makeSearchGrid(0_deg, 0_deg, 10_deg));
    scheduler.next(0ms);
    scheduler.next(10ms);
    scheduler.addRequest(makeRequest(beam::RequestBeamType::TRACKING, 1000ms, 6, 1_deg, 2_deg));

    scheduler.setOperationState(beam::OperationState::OFF);
    scheduler.setOperationState(beam::OperationState::ON);

    // 탐색 격자를 다시 받기 전에는 준비 상태가 아니다.
    EXPECT_FALSE(scheduler.next(20ms).has_value());

    // 새 운용에서는 새로 받은 격자(영 자세), 처음 구역, 카운트 1부터 시작한다.
    scheduler.setSearchGrid(makeZeroSearchGrid());
    const auto first = scheduler.next(1000ms);

    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(first->beamType, beam::BeamType::SEARCH);
    EXPECT_EQ(first->beamId, 1u);
    EXPECT_EQ(first->commandCount, 1u);
    EXPECT_TRUE(first->direction_ant.azimuth.isEquivalentTo(gridAzimuth(0)));
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
// 빔 폭은 요청을 만들 때 팩토리가 정한다. Scheduler는 설정을 직접 읽지 않는다.
// 설정 값 검증(빔폭 0 이하)은 config_test.cpp 에서 확인한다.

TEST(SchedulerTest, RequestBeamWidthFollowsEachRequest)
{
    const beam::BeamRequestFactory factory(
        beam::RequestBeamConfig::Builder()
            .confirmationBeamWidth(beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)))
            .trackingBeamWidth(beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)))
            .confirmationBeamWidth(beam::BeamWidth(
                math::AngularWidth(2_deg),
                math::AngularWidth(3_deg)))
            .trackingBeamWidth(beam::BeamWidth(
                math::AngularWidth(4_deg),
                math::AngularWidth(5_deg)))
            .build());
    beam::Scheduler scheduler;
    makeReady(scheduler);

    scheduler.addRequest(makeRequest(factory, beam::RequestBeamType::CONFIRMATION, 0ms, 1, 1_deg, 5_deg));
    const auto confirmation = scheduler.next(0ms);
    ASSERT_TRUE(confirmation.has_value());
    EXPECT_EQ(confirmation->beamType, beam::BeamType::CONFIRMATION);
    EXPECT_EQ(confirmation->beamWidth.azimuth, 2_deg);
    EXPECT_EQ(confirmation->beamWidth.elevation, 3_deg);

    scheduler.addRequest(makeRequest(factory, beam::RequestBeamType::TRACKING, 10ms, 2, 1_deg, 5_deg));
    const auto tracking = scheduler.next(10ms);
    ASSERT_TRUE(tracking.has_value());
    EXPECT_EQ(tracking->beamType, beam::BeamType::TRACKING);
    EXPECT_EQ(tracking->beamWidth.azimuth, 4_deg);
    EXPECT_EQ(tracking->beamWidth.elevation, 5_deg);
}
