#include "scheduler/scheduler.hpp"
#include "scheduler/attitude_frame.hpp"

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
    math::CircularAngle(-42_deg),
    4.2_deg,
    math::TiltAngle(3_deg),
    4.4_deg,
    beam::BeamWidth(6_deg, 6_deg));

// 기본 추적 영역: 안테나 기준 방위각 -60~60도, 고각 -10~60도
beam::RequestCoverageConfig makeCoverageConfig()
{
    return beam::RequestCoverageConfig(math::CircularAngleRange(-60_deg, 60_deg), math::AngleRange(-10_deg, 60_deg));
}

// 자세정보로 탐색 격자와 영역 판정을 만드는 일은 BeamController가 맡으므로, 테스트에서는 직접 만들어 넘긴다.
beam::AttitudeFrame makeAttitudeFrame(math::Angle roll, math::Angle pitch, math::Angle yaw)
{
    const beam::AntEnuTransform transform(
        beam::RadarAttitude(roll, math::TiltAngle(pitch), yaw),
        beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}));
    return beam::AttitudeFrame(transform, GRID_CONFIG, makeCoverageConfig());
}

beam::AttitudeFrame makeZeroAttitudeFrame()
{
    return makeAttitudeFrame(0_deg, 0_deg, 0_deg);
}

// ON으로 바꾸고 AttitudeFrame까지 넣어 준비 상태로 만든다.
void makeReady(beam::Scheduler& scheduler, beam::AttitudeFrame attitudeFrame)
{
    scheduler.setOperationState(beam::OperationState::ON);
    scheduler.setAttitudeFrame(std::move(attitudeFrame));
}

void makeReady(beam::Scheduler& scheduler)
{
    makeReady(scheduler, makeZeroAttitudeFrame());
}

// 큐에 전달할 요청의 빔 폭과 우선순위를 테스트 입력으로 직접 지정한다.
beam::BeamRequest makeRequest(
    beam::BeamType beamType,
    std::chrono::milliseconds transmitTime,
    std::uint32_t requestId,
    math::Angle azimuth,
    math::Angle elevation
) {
    const int priority = beamType == beam::BeamType::CONFIRMATION ? 1 : 0;
    return beam::BeamRequest(
        beamType, beam::BeamWidth(3_deg, 3_deg), priority,
        transmitTime, requestId, math::SphericalDirection(azimuth, elevation));
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

// 스케줄러는 초기 상태에서 송신 명령을 반환하지 않는다.
TEST(SchedulerTest, ReturnsNothingInitially)
{
    // Given: 검사할 운용 상태의 스케줄러를 준비한다.
    beam::Scheduler scheduler;

    // When: 운용 입력을 적용하고 송신 명령을 요청한다.
    const auto command = scheduler.next(0ms);

    // Then: 스케줄러는 초기 상태에서 송신 명령을 반환하지 않는다.
    EXPECT_FALSE(command.has_value());
}

// 스케줄러는 켜져 있어도 자세 프레임이 없으면 송신하지 않는다.
TEST(SchedulerTest, ReturnsNothingWhenOnButAttitudeFrameNotReceived)
{
    // Given: 검사할 운용 상태의 스케줄러를 준비한다.
    beam::Scheduler scheduler;
    scheduler.setOperationState(beam::OperationState::ON);

    // When: 운용 입력을 적용하고 송신 명령을 요청한다.
    const auto command = scheduler.next(0ms);

    // Then: 스케줄러는 켜져 있어도 자세 프레임이 없으면 송신하지 않는다.
    EXPECT_FALSE(command.has_value());
}

// 꺼진 상태에서 받은 자세 프레임은 준비 상태를 만들지 않는다.
TEST(SchedulerTest, AttitudeFrameReceivedWhileOffIsIgnored)
{
    // Given: 검사할 운용 상태의 스케줄러를 준비한다.
    beam::Scheduler scheduler;
    scheduler.setAttitudeFrame(makeZeroAttitudeFrame());

    // When: 운용 입력을 적용하고 송신 명령을 요청한다.
    scheduler.setOperationState(beam::OperationState::ON);
    const auto command = scheduler.next(0ms);

    // Then: 꺼진 상태에서 받은 자세 프레임은 준비 상태를 만들지 않는다.
    EXPECT_FALSE(command.has_value());
}

// 켜진 뒤 자세 프레임을 받으면 스케줄러가 송신할 수 있다.
TEST(SchedulerTest, AttitudeFrameAfterOnMakesSchedulerReady)
{
    // Given: 검사할 운용 상태의 스케줄러를 준비한다.
    beam::Scheduler scheduler;
    scheduler.setOperationState(beam::OperationState::ON);

    // When: 운용 입력을 적용하고 송신 명령을 요청한다.
    scheduler.setAttitudeFrame(makeZeroAttitudeFrame());
    const auto command = scheduler.next(0ms);

    // Then: 켜진 뒤 자세 프레임을 받으면 스케줄러가 송신할 수 있다.
    EXPECT_TRUE(command.has_value());
}

// 스케줄러는 운용을 끄면 송신을 중단한다.
TEST(SchedulerTest, ReturnsNothingAfterTurnedOff)
{
    // Given: 검사에 필요한 운용 상태와 요청 또는 자세 프레임을 준비한다.
    beam::Scheduler scheduler;
    makeReady(scheduler);
    ASSERT_TRUE(scheduler.next(0ms).has_value());

    // When: 준비된 스케줄러의 운용을 끈다.
    scheduler.setOperationState(beam::OperationState::OFF);

    // Then: 스케줄러는 운용을 끄면 송신을 중단한다.
    EXPECT_FALSE(scheduler.next(10ms).has_value());
}

// 스케줄러는 운용 중 두 번째 자세 프레임을 무시한다.
TEST(SchedulerTest, SecondAttitudeFrameIsIgnored)
{
    // Given: 검사에 필요한 운용 상태와 요청 또는 자세 프레임을 준비한다.
    beam::Scheduler scheduler;
    makeReady(scheduler, makeZeroAttitudeFrame());

    // When: 다른 자세 프레임을 다시 전달하고 명령을 꺼낸다.
    scheduler.setAttitudeFrame(makeAttitudeFrame(0_deg, 0_deg, 10_deg));
    const auto command = scheduler.next(0ms);

    // Then: 스케줄러는 운용 중 두 번째 자세 프레임을 무시한다.
    ASSERT_TRUE(command.has_value());
    EXPECT_TRUE(command->direction_ant.azimuth.isEquivalentTo(gridAzimuth(0)));
}

// ---------- 탐색 빔 ----------

// 요청이 없는 스케줄러는 격자 순서로 탐색 빔을 송신한다.
TEST(SchedulerTest, ReturnsSearchBeamsInGridOrderWhenNoRequests)
{
    // Given: 검사에 필요한 운용 상태와 요청 또는 자세 프레임을 준비한다.
    beam::Scheduler scheduler;
    makeReady(scheduler);

    // When: 연속된 틱에서 송신 명령을 꺼낸다.
    for (std::size_t index = 0; index < 30; ++index)
    {
        const auto command = scheduler.next(std::chrono::milliseconds(10 * index));

        // Then: 요청이 없는 스케줄러는 격자 순서로 탐색 빔을 송신한다.
        ASSERT_TRUE(command.has_value()) << "index=" << index;
        EXPECT_EQ(command->beamType, beam::BeamType::SEARCH) << "index=" << index;
        EXPECT_EQ(command->beamId, static_cast<std::uint32_t>(index + 1)) << "index=" << index;
        EXPECT_TRUE(command->direction_ant.azimuth.isEquivalentTo(gridAzimuth(index))) << "index=" << index;
        EXPECT_EQ(command->direction_ant.elevation, gridElevation(index)) << "index=" << index;
        EXPECT_EQ(command->beamWidth.azimuth, 6_deg);
        EXPECT_EQ(command->beamWidth.elevation, 6_deg);
    }
}

// 스케줄러는 마지막 탐색 빔 다음에 첫 빔을 송신한다.
TEST(SchedulerTest, SearchCursorWrapsAroundAfterLastBeam)
{
    // Given: 검사에 필요한 운용 상태와 요청 또는 자세 프레임을 준비한다.
    beam::Scheduler scheduler;
    makeReady(scheduler);

    // When: 한 바퀴의 탐색 명령을 송신한 뒤 다음 명령을 꺼낸다.
    const std::size_t beamCount = GRID_CONFIG.azimuthCount() * GRID_CONFIG.elevationCount();
    for (std::size_t index = 0; index < beamCount; ++index)
    {

        // Then: 스케줄러는 마지막 탐색 빔 다음에 첫 빔을 송신한다.
        ASSERT_TRUE(scheduler.next(std::chrono::milliseconds(10 * index)).has_value());
    }

    const auto wrapped = scheduler.next(std::chrono::milliseconds(10 * beamCount));

    ASSERT_TRUE(wrapped.has_value());
    EXPECT_EQ(wrapped->beamId, 1u);
}

// 탐색 명령의 송신 시각은 해당 틱의 현재 시각이다.
TEST(SchedulerTest, SearchBeamTransmitTimeIsCurrentTime)
{
    // Given: 송신 준비가 된 스케줄러를 준비한다.
    beam::Scheduler scheduler;
    makeReady(scheduler);

    // When: 서로 다른 두 틱의 송신 명령을 꺼낸다.
    const auto first = scheduler.next(1230ms);
    const auto second = scheduler.next(1240ms);

    // Then: 각 명령이 해당 틱의 현재 시각을 갖는다.
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(first->transmitTime, 1230ms);
    EXPECT_EQ(second->transmitTime, 1240ms);
}

// 스케줄러는 전달받은 자세 프레임의 탐색 방향을 송신한다.
TEST(SchedulerTest, SearchBeamsFollowGivenSearchGrid)
{
    // Given: 검사에 필요한 운용 상태와 요청 또는 자세 프레임을 준비한다.
    // 격자는 안테나 정면 기준이므로 안테나가 yaw 만큼 돌아가 있으면 ENU 방위각도 +yaw 만큼 이동한다.
    beam::Scheduler scheduler;
    makeReady(scheduler, makeAttitudeFrame(0_deg, 0_deg, 10_deg));

    // When: 탐색 송신 명령을 꺼낸다.
    const auto command = scheduler.next(0ms);

    // Then: 스케줄러는 전달받은 자세 프레임의 탐색 방향을 송신한다.
    ASSERT_TRUE(command.has_value());
    EXPECT_TRUE(command->direction_ant.azimuth.isEquivalentTo(gridAzimuth(0) + 10_deg));
    EXPECT_EQ(command->direction_ant.elevation, gridElevation(0));
}

// ---------- 요청 빔 ----------

// 스케줄러는 확인 요청을 확인 빔 명령으로 송신한다.
TEST(SchedulerTest, ConfirmationRequestProducesConfirmationBeam)
{
    // Given: 검사에 필요한 운용 상태와 요청 또는 자세 프레임을 준비한다.
    beam::Scheduler scheduler;
    makeReady(scheduler);
    scheduler.addRequest(makeRequest(beam::BeamType::CONFIRMATION, 100ms, 20, 12.5_deg, 20.5_deg));

    // When: 확인 요청의 예정 시각에 송신 명령을 꺼낸다.
    const auto command = scheduler.next(100ms);

    // Then: 스케줄러는 확인 요청을 확인 빔 명령으로 송신한다.
    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamType::CONFIRMATION);
    EXPECT_EQ(command->beamId, 20u);
    EXPECT_EQ(command->transmitTime, 100ms);
    EXPECT_TRUE(command->direction_ant.azimuth.isEquivalentTo(12.5_deg));
    EXPECT_EQ(command->direction_ant.elevation, 20.5_deg);
    EXPECT_EQ(command->beamWidth.azimuth, 3_deg);
    EXPECT_EQ(command->beamWidth.elevation, 3_deg);
}

// 스케줄러는 추적 요청을 추적 빔 명령으로 송신한다.
TEST(SchedulerTest, TrackingRequestProducesTrackingBeam)
{
    // Given: 검사에 필요한 운용 상태와 요청 또는 자세 프레임을 준비한다.
    beam::Scheduler scheduler;
    makeReady(scheduler);
    scheduler.addRequest(makeRequest(beam::BeamType::TRACKING, 100ms, 100, -30_deg, 5_deg));

    // When: 추적 요청의 예정 시각에 송신 명령을 꺼낸다.
    const auto command = scheduler.next(100ms);

    // Then: 스케줄러는 추적 요청을 추적 빔 명령으로 송신한다.
    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamType::TRACKING);
    EXPECT_EQ(command->beamId, 100u);
    EXPECT_TRUE(command->direction_ant.azimuth.isEquivalentTo(-30_deg));
    EXPECT_EQ(command->direction_ant.elevation, 5_deg);
    EXPECT_EQ(command->beamWidth.azimuth, 3_deg);
    EXPECT_EQ(command->beamWidth.elevation, 3_deg);
}

// 미래 요청의 예정 시각까지 탐색 빔을 송신한다.
TEST(SchedulerTest, FutureRequestKeepsSearchingUntilItsTime)
{
    // Given: 미래 요청이 들어 있는 준비된 스케줄러를 만든다.
    beam::Scheduler scheduler;
    makeReady(scheduler);
    scheduler.addRequest(makeRequest(beam::BeamType::TRACKING, 30ms, 20, 1_deg, 2_deg));

    // When: 요청 시각 전부터 예정 시각까지 송신 명령을 꺼낸다.
    const auto at0 = scheduler.next(0ms);
    const auto at10 = scheduler.next(10ms);
    const auto at20 = scheduler.next(20ms);
    const auto at30 = scheduler.next(30ms);

    // Then: 예정 시각 전에는 탐색 빔, 예정 시각에는 요청 빔이 송신된다.
    ASSERT_TRUE(at0.has_value());
    ASSERT_TRUE(at10.has_value());
    ASSERT_TRUE(at20.has_value());
    ASSERT_TRUE(at30.has_value());
    EXPECT_EQ(at0->beamType, beam::BeamType::SEARCH);
    EXPECT_EQ(at10->beamType, beam::BeamType::SEARCH);
    EXPECT_EQ(at20->beamType, beam::BeamType::SEARCH);
    EXPECT_EQ(at30->beamType, beam::BeamType::TRACKING);
}

// 요청 빔 송신은 탐색 커서를 진행시키지 않는다.
TEST(SchedulerTest, RequestBeamDoesNotAdvanceSearchCursor)
{
    // Given: 탐색 빔 두 개를 송신한 뒤 확인 요청을 준비한다.
    beam::Scheduler scheduler;
    makeReady(scheduler);
    scheduler.next(0ms);
    scheduler.next(10ms);
    scheduler.addRequest(makeRequest(beam::BeamType::CONFIRMATION, 20ms, 10, 1_deg, 2_deg));

    // When: 확인 요청과 그 다음 탐색 명령을 꺼낸다.
    const auto request = scheduler.next(20ms);
    const auto search = scheduler.next(30ms);

    // Then: 요청 빔 뒤의 탐색은 세 번째 빔부터 이어진다.
    ASSERT_TRUE(request.has_value());
    EXPECT_EQ(request->beamType, beam::BeamType::CONFIRMATION);
    ASSERT_TRUE(search.has_value());
    EXPECT_EQ(search->beamType, beam::BeamType::SEARCH);
    EXPECT_EQ(search->beamId, 3u);
}

// 지연된 요청 명령의 송신 시각은 현재 틱의 시각이다.
TEST(SchedulerTest, DelayedRequestUsesCurrentTimeAsTransmitTime)
{
    // Given: 검사에 필요한 운용 상태와 요청 또는 자세 프레임을 준비한다.
    beam::Scheduler scheduler;
    makeReady(scheduler);
    scheduler.addRequest(makeRequest(beam::BeamType::TRACKING, 1005ms, 10, 1_deg, 2_deg));

    // When: 요청 시각을 지난 틱에서 송신 명령을 꺼낸다.
    const auto command = scheduler.next(1010ms);

    // Then: 지연된 요청 명령의 송신 시각은 현재 틱의 시각이다.
    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamType::TRACKING);
    EXPECT_EQ(command->transmitTime, 1010ms);
}

// 오래된 요청도 송신되고 다음 탐색 순서는 유지된다.
TEST(SchedulerTest, VeryOldRequestIsStillSentAsRequestBeam)
{
    // Given: 송신 시각이 오래 지난 요청을 가진 스케줄러를 준비한다.
    beam::Scheduler scheduler;
    makeReady(scheduler);
    scheduler.addRequest(makeRequest(beam::BeamType::TRACKING, 1000ms, 7, 1_deg, 2_deg));

    // When: 예정 시각보다 훨씬 늦게 두 틱의 송신 명령을 꺼낸다.
    const auto request = scheduler.next(10000ms);
    const auto search = scheduler.next(10010ms);

    // Then: 요청은 폐기되지 않고 현재 시각에 송신되며 탐색 순서도 유지된다.
    ASSERT_TRUE(request.has_value());
    EXPECT_EQ(request->beamType, beam::BeamType::TRACKING);
    EXPECT_EQ(request->beamId, 7u);
    EXPECT_EQ(request->transmitTime, 10000ms);
    ASSERT_TRUE(search.has_value());
    EXPECT_EQ(search->beamType, beam::BeamType::SEARCH);
    EXPECT_EQ(search->beamId, 1u);
}

// 스케줄러는 몰린 요청을 차례로 송신하고 탐색을 이어간다.
TEST(SchedulerTest, CrowdedTicksFollowDesignedScenario)
{
    // Given: 검사에 필요한 운용 상태와 요청 또는 자세 프레임을 준비한다.
    // 1005 추적 A, 1010 확인 B, 1010 추적 C, 1010 추적 D
    beam::Scheduler scheduler;
    makeReady(scheduler);
    scheduler.addRequest(makeRequest(beam::BeamType::TRACKING, 1005ms, 1, 1_deg, 0_deg));
    scheduler.addRequest(makeRequest(beam::BeamType::CONFIRMATION, 1010ms, 2, 2_deg, 0_deg));
    scheduler.addRequest(makeRequest(beam::BeamType::TRACKING, 1010ms, 3, 3_deg, 0_deg));
    scheduler.addRequest(makeRequest(beam::BeamType::TRACKING, 1010ms, 4, 4_deg, 0_deg));

    // When: 요청 전후의 연속된 틱에서 송신 명령을 꺼낸다.
    const auto at1000 = scheduler.next(1000ms);
    const auto at1010 = scheduler.next(1010ms);
    const auto at1020 = scheduler.next(1020ms);
    const auto at1030 = scheduler.next(1030ms);
    const auto at1040 = scheduler.next(1040ms);
    const auto at1050 = scheduler.next(1050ms);

    // Then: 몰린 요청은 순서대로 송신되고 탐색은 이어진다.
    ASSERT_TRUE(at1000.has_value());
    ASSERT_TRUE(at1010.has_value());
    ASSERT_TRUE(at1020.has_value());
    ASSERT_TRUE(at1030.has_value());
    ASSERT_TRUE(at1040.has_value());
    ASSERT_TRUE(at1050.has_value());
    EXPECT_EQ(at1000->beamType, beam::BeamType::SEARCH);
    EXPECT_EQ(at1000->beamId, 1u);

    EXPECT_EQ(at1010->beamType, beam::BeamType::TRACKING);
    EXPECT_TRUE(at1010->direction_ant.azimuth.isEquivalentTo(1_deg));

    EXPECT_EQ(at1020->beamType, beam::BeamType::CONFIRMATION);
    EXPECT_TRUE(at1020->direction_ant.azimuth.isEquivalentTo(2_deg));

    EXPECT_EQ(at1030->beamType, beam::BeamType::TRACKING);
    EXPECT_TRUE(at1030->direction_ant.azimuth.isEquivalentTo(3_deg));

    // D는 1010ms 요청이 30ms 지연되어 1040ms에 송신된다. 폐기되지 않는다.
    EXPECT_EQ(at1040->beamType, beam::BeamType::TRACKING);
    EXPECT_TRUE(at1040->direction_ant.azimuth.isEquivalentTo(4_deg));

    EXPECT_EQ(at1050->beamType, beam::BeamType::SEARCH);
    EXPECT_EQ(at1050->beamId, 2u);
}

// ---------- 명령 카운트 ----------

// 송신 명령 수는 빔 종류와 관계없이 1부터 연속 증가한다.
TEST(SchedulerTest, CommandCountStartsAtOneAndIncreasesPerCommand)
{
    // Given: 탐색 중간에 요청을 송신할 스케줄러를 준비한다.
    beam::Scheduler scheduler;
    makeReady(scheduler);
    scheduler.addRequest(makeRequest(beam::BeamType::TRACKING, 20ms, 100, 1_deg, 2_deg));

    // When: 탐색과 요청이 섞인 네 틱의 명령을 꺼낸다.
    const auto at0 = scheduler.next(0ms);
    const auto at10 = scheduler.next(10ms);
    const auto at20 = scheduler.next(20ms);
    const auto at30 = scheduler.next(30ms);

    // Then: 각 명령의 수가 1부터 4까지 연속 증가한다.
    ASSERT_TRUE(at0.has_value());
    ASSERT_TRUE(at10.has_value());
    ASSERT_TRUE(at20.has_value());
    ASSERT_TRUE(at30.has_value());
    EXPECT_EQ(at0->commandCount, 1u);
    EXPECT_EQ(at10->commandCount, 2u);
    EXPECT_EQ(at20->commandCount, 3u);
    EXPECT_EQ(at30->commandCount, 4u);
}

// 송신하지 않은 틱은 명령 수를 증가시키지 않는다.
TEST(SchedulerTest, CommandCountDoesNotIncreaseWhenNothingIsSent)
{
    // Given: 켜져 있지만 자세 프레임은 없는 스케줄러를 준비한다.
    beam::Scheduler scheduler;
    scheduler.setOperationState(beam::OperationState::ON);

    // When: 준비 전의 틱을 처리한 뒤 자세 프레임을 받아 다음 틱을 처리한다.
    const auto notReady = scheduler.next(0ms);
    scheduler.setAttitudeFrame(makeZeroAttitudeFrame());
    const auto first = scheduler.next(10ms);

    // Then: 준비 전에는 송신하지 않고 첫 송신 명령 수는 1이다.
    EXPECT_FALSE(notReady.has_value());
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(first->commandCount, 1u);
}

// ---------- 요청 접수 조건 ----------

// 꺼진 상태에서 받은 요청은 이후 운용에 남지 않는다.
TEST(SchedulerTest, RequestWhileOffIsIgnored)
{
    // Given: 검사할 운용 상태의 스케줄러를 준비한다.
    beam::Scheduler scheduler;
    const auto request = makeRequest(beam::BeamType::TRACKING, 0ms, 3, 1_deg, 2_deg);

    // When: 운용 입력을 적용하고 송신 명령을 요청한다.
    scheduler.addRequest(request);
    makeReady(scheduler);
    const auto command = scheduler.next(0ms);

    // Then: 꺼진 상태에서 받은 요청은 이후 운용에 남지 않는다.
    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamType::SEARCH);
}

// 자세 프레임 전에 받은 요청은 이후 송신되지 않는다.
TEST(SchedulerTest, RequestBeforeAttitudeFrameIsIgnored)
{
    // Given: 검사할 운용 상태의 스케줄러를 준비한다.
    beam::Scheduler scheduler;
    scheduler.setOperationState(beam::OperationState::ON);

    // When: 운용 입력을 적용하고 송신 명령을 요청한다.
    scheduler.addRequest(makeRequest(beam::BeamType::TRACKING, 0ms, 4, 1_deg, 2_deg));
    scheduler.setAttitudeFrame(makeZeroAttitudeFrame());
    const auto command = scheduler.next(0ms);

    // Then: 자세 프레임 전에 받은 요청은 이후 송신되지 않는다.
    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamType::SEARCH);
}

// 꺼져 있거나 자세 프레임이 없으면 요청 접수 상태는 준비 전이다.
TEST(SchedulerTest, AddRequestReturnsNotReadyWhileOffOrBeforeAttitudeFrame)
{
    // Given: 꺼진 스케줄러와 접수할 요청을 준비한다.
    beam::Scheduler scheduler;
    const auto request = makeRequest(beam::BeamType::TRACKING, 0ms, 3, 1_deg, 2_deg);

    // When: 꺼진 상태와 켜진 뒤 자세 프레임을 받기 전 상태에서 요청한다.
    const auto whileOff = scheduler.addRequest(request);
    scheduler.setOperationState(beam::OperationState::ON);
    const auto beforeAttitude = scheduler.addRequest(request);

    // Then: 두 상태 모두 준비 전 결과를 반환한다.
    EXPECT_EQ(whileOff, beam::RequestBeamStatus::NOT_READY);
    EXPECT_EQ(beforeAttitude, beam::RequestBeamStatus::NOT_READY);
}

// 준비된 스케줄러는 요청을 접수하고 송신한다.
TEST(SchedulerTest, AddRequestReturnsAcceptedWhenQueued)
{
    // Given: 송신 준비가 된 스케줄러와 영역 안의 요청을 준비한다.
    beam::Scheduler scheduler;
    makeReady(scheduler);
    const auto request = makeRequest(beam::BeamType::TRACKING, 0ms, 5, 1_deg, 2_deg);

    // When: 요청을 접수하고 송신 명령을 꺼낸다.
    const auto status = scheduler.addRequest(request);
    const auto command = scheduler.next(0ms);

    // Then: 접수 성공을 반환하고 해당 요청을 송신한다.
    EXPECT_EQ(status, beam::RequestBeamStatus::ACCEPTED);
    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamType::TRACKING);
    EXPECT_EQ(command->beamId, request.requestId());
}

// ---------- 추적 영역 판정 ----------

// 스케줄러는 영역 밖 요청을 거부하고 탐색을 계속한다.
TEST(SchedulerTest, RequestOutsideCoverageIsNotQueued)
{
    // Given: 검사할 자세 프레임을 받은 스케줄러를 준비한다.
    beam::Scheduler scheduler;
    makeReady(scheduler);

    // 확인·추적 빔 모두 같은 영역으로 판정한다.

    // When: 영역 안팎의 요청을 접수한 뒤 송신 명령을 꺼낸다.
    const auto status1 = scheduler.addRequest(makeRequest(beam::BeamType::TRACKING, 0ms, 6, 61_deg, 10_deg));
    const auto status2 = scheduler.addRequest(makeRequest(beam::BeamType::CONFIRMATION, 0ms, 7, 0_deg, -11_deg));

    const auto command = scheduler.next(0ms);

    // Then: 스케줄러는 영역 밖 요청을 거부하고 탐색을 계속한다.
    EXPECT_EQ(status1, beam::RequestBeamStatus::OUT_OF_COVERAGE);
    EXPECT_EQ(status2, beam::RequestBeamStatus::OUT_OF_COVERAGE);
    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamType::SEARCH);
    EXPECT_EQ(command->beamId, 1u);
}

// 스케줄러는 받은 자세 프레임의 영역 판정으로 요청을 접수한다.
TEST(SchedulerTest, CoverageIsJudgedInAntennaFrameOfReceivedAttitude)
{
    // Given: 검사할 자세 프레임을 받은 스케줄러를 준비한다.
    beam::Scheduler scheduler;
    makeReady(scheduler, makeAttitudeFrame(0_deg, 0_deg, 13.02429_deg));

    // ENU 75도 → 안테나 약 62도: 밖

    // When: 영역 안팎의 요청을 접수한 뒤 송신 명령을 꺼낸다.
    const auto status1 = scheduler.addRequest(makeRequest(beam::BeamType::TRACKING, 0ms, 8, 75_deg, 0_deg));
    // ENU 72도 → 안테나 약 59도: 안. 송신 방향은 요청 그대로(ENU)다.
    const auto status2 = scheduler.addRequest(makeRequest(beam::BeamType::TRACKING, 0ms, 9, 72_deg, 0_deg));

    const auto command = scheduler.next(0ms);

    // Then: 스케줄러는 받은 자세 프레임의 영역 판정으로 요청을 접수한다.
    EXPECT_EQ(status1, beam::RequestBeamStatus::OUT_OF_COVERAGE);
    EXPECT_EQ(status2, beam::RequestBeamStatus::ACCEPTED);
    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamType::TRACKING);
    EXPECT_EQ(command->beamId, 9u);
    EXPECT_TRUE(command->direction_ant.azimuth.isEquivalentTo(72_deg));
}

// 스케줄러는 재시작 후 새 자세 프레임의 영역 판정을 사용한다.
TEST(SchedulerTest, CoverageFollowsAttitudeFrameReceivedAfterRestart)
{
    // Given: 회전한 자세 프레임을 받은 스케줄러와 영역 밖 요청을 준비한다.
    beam::Scheduler scheduler;
    makeReady(scheduler, makeAttitudeFrame(0_deg, 0_deg, 13.02429_deg));
    const auto request = makeRequest(beam::BeamType::TRACKING, 0ms, 10, -50_deg, 0_deg);
    ASSERT_EQ(scheduler.addRequest(request), beam::RequestBeamStatus::OUT_OF_COVERAGE);

    // When: 운용을 재시작하고 새 자세 프레임 전후에 같은 방향을 요청한다.
    scheduler.setOperationState(beam::OperationState::OFF);
    scheduler.setOperationState(beam::OperationState::ON);
    const auto beforeAttitude = scheduler.addRequest(request);
    scheduler.setAttitudeFrame(makeZeroAttitudeFrame());
    const auto afterAttitude = scheduler.addRequest(request);

    // Then: 새 프레임 전에는 준비 전이며 새 프레임의 영역 안인 요청은 접수한다.
    EXPECT_EQ(beforeAttitude, beam::RequestBeamStatus::NOT_READY);
    EXPECT_EQ(afterAttitude, beam::RequestBeamStatus::ACCEPTED);
}

// ---------- 초기화 ----------

// 운용 재시작은 요청과 자세 프레임 및 탐색 순서와 명령 수를 초기화한다.
TEST(SchedulerTest, TurningOnAgainAfterOffResetsEverything)
{
    // Given: 탐색을 진행하고 미래 요청도 접수한 스케줄러를 준비한다.
    beam::Scheduler scheduler;
    makeReady(scheduler, makeAttitudeFrame(0_deg, 0_deg, 10_deg));
    scheduler.next(0ms);
    scheduler.next(10ms);
    scheduler.addRequest(makeRequest(beam::BeamType::TRACKING, 1000ms, 6, 1_deg, 2_deg));

    // When: 운용을 재시작하고 새 자세 프레임 전후의 송신 명령을 꺼낸다.
    scheduler.setOperationState(beam::OperationState::OFF);
    scheduler.setOperationState(beam::OperationState::ON);
    const auto beforeAttitude = scheduler.next(20ms);
    scheduler.setAttitudeFrame(makeZeroAttitudeFrame());
    const auto first = scheduler.next(1000ms);

    // Then: 새 자세를 받아야 송신하며 이전 요청 없이 탐색과 명령 수가 처음부터 시작한다.
    EXPECT_FALSE(beforeAttitude.has_value());
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(first->beamType, beam::BeamType::SEARCH);
    EXPECT_EQ(first->beamId, 1u);
    EXPECT_EQ(first->commandCount, 1u);
    EXPECT_TRUE(first->direction_ant.azimuth.isEquivalentTo(gridAzimuth(0)));
}

// 켜진 상태에서 다시 켜도 탐색 순서와 명령 수는 유지된다.
TEST(SchedulerTest, TurningOnWhileAlreadyOnKeepsState)
{
    // Given: 검사에 필요한 운용 상태와 요청 또는 자세 프레임을 준비한다.
    beam::Scheduler scheduler;
    makeReady(scheduler);
    scheduler.next(0ms);

    // When: 켜진 스케줄러에 켜기 명령을 다시 전달한다.
    scheduler.setOperationState(beam::OperationState::ON);

    const auto command = scheduler.next(10ms);

    // Then: 켜진 상태에서 다시 켜도 탐색 순서와 명령 수는 유지된다.
    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamId, 2u);
    EXPECT_EQ(command->commandCount, 2u);
}

// 꺼진 상태에서 다시 꺼도 이후 정상 운용할 수 있다.
TEST(SchedulerTest, TurningOffWhileAlreadyOffDoesNothing)
{
    // Given: 검사할 운용 상태의 스케줄러를 준비한다.
    beam::Scheduler scheduler;

    // When: 운용 입력을 적용하고 송신 명령을 요청한다.
    scheduler.setOperationState(beam::OperationState::OFF);
    makeReady(scheduler);
    const auto command = scheduler.next(0ms);

    // Then: 꺼진 상태에서 다시 꺼도 이후 정상 운용할 수 있다.
    EXPECT_TRUE(command.has_value());
}

// ---------- 요청 빔 설정 반영 ----------
// Scheduler는 요청에 담긴 빔 폭을 송신 명령에 전달한다.

// 스케줄러는 각 요청의 빔 폭을 송신 명령에 전달한다.
TEST(SchedulerTest, RequestBeamWidthFollowsEachRequest)
{
    // Given: 빔 폭이 서로 다른 두 요청을 준비된 스케줄러에 넣는다.
    beam::Scheduler scheduler;
    makeReady(scheduler);
    scheduler.addRequest(beam::BeamRequest(
        beam::BeamType::CONFIRMATION, beam::BeamWidth(2_deg, 3_deg), 1,
        0ms, 1, math::SphericalDirection(1_deg, 5_deg)));
    scheduler.addRequest(beam::BeamRequest(
        beam::BeamType::TRACKING, beam::BeamWidth(4_deg, 5_deg), 0,
        10ms, 2, math::SphericalDirection(1_deg, 5_deg)));

    // When: 각 요청의 송신 시각에 명령을 꺼낸다.
    const auto confirmation = scheduler.next(0ms);
    const auto tracking = scheduler.next(10ms);

    // Then: 각 명령이 해당 요청의 빔 폭을 유지한다.
    ASSERT_TRUE(confirmation.has_value());
    EXPECT_EQ(confirmation->beamType, beam::BeamType::CONFIRMATION);
    EXPECT_EQ(confirmation->beamWidth.azimuth, 2_deg);
    EXPECT_EQ(confirmation->beamWidth.elevation, 3_deg);
    ASSERT_TRUE(tracking.has_value());
    EXPECT_EQ(tracking->beamType, beam::BeamType::TRACKING);
    EXPECT_EQ(tracking->beamWidth.azimuth, 4_deg);
    EXPECT_EQ(tracking->beamWidth.elevation, 5_deg);
}
