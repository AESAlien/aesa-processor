#include <target/config/target_config.hpp>
#include <target/controller/target_controller.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace target
{
namespace
{

using namespace math::literals;
using namespace std::chrono_literals;
using Time = std::chrono::milliseconds;
using BeamType = beam::BeamType;

math::Distance range(std::uint32_t id)
{
    return math::Distance(1000_m + 100_m * static_cast<double>(id - 1));
}

DetectionEvent result(BeamType type, std::uint32_t id, Time time, bool detected = true)
{
    std::vector<DetectionEvent::Plot> plots;
    if (detected)
    {
        plots.emplace_back(range(id), math::CircularAngle(0_deg), math::TiltAngle(0_deg), 0_mps, -10.0f);
    }
    return DetectionEvent(type, id, time, std::move(plots));
}

void addCandidates(TargetController& controller, std::uint32_t count)
{
    // 이후 시각의 확정 관측을 기준으로 이동 허용 거리가 적용되지 않도록 탐색 결과를 먼저 등록한다.
    for (std::uint32_t id = 1; id <= count; ++id)
    {
        const auto request = controller.handleDetection(result(BeamType::SEARCH, id, 0ms));
        ASSERT_TRUE(request) << "Candidate " << id;
        EXPECT_EQ(request->beamType, beam::RequestBeamType::CONFIRMATION);
        EXPECT_EQ(request->requestId, id);
        EXPECT_EQ(request->transmitTime, 100ms);
    }
}

void confirm(TargetController& controller, std::uint32_t first, std::uint32_t last)
{
    for (auto id = first; id <= last; ++id)
    {
        const auto request = controller.handleDetection(result(BeamType::CONFIRMATION, id, 100ms));
        ASSERT_TRUE(request) << "Confirmation " << id;
        EXPECT_EQ(request->beamType, beam::RequestBeamType::TRACKING);
        EXPECT_EQ(request->requestId, id);
        EXPECT_EQ(request->transmitTime, 1100ms);
    }
}

void expectReportedIds(
    const TargetController& controller,
    std::size_t count,
    std::uint16_t first
) {
    const auto events = controller.nextTrackEvents();
    ASSERT_EQ(events.size(), count);
    for (std::size_t index = 0; index < count; ++index)
    {
        const auto id = static_cast<std::uint16_t>(static_cast<std::size_t>(first) + index);
        EXPECT_EQ(events[index].id, id);
        EXPECT_EQ(events[index].state, TrackEvent::TrackStatus::TRACKING);
        EXPECT_EQ(events[index].timestamp, 100ms);
        EXPECT_NEAR(events[index].slantRange.m(), range(id).m(), 1e-9);
    }
}

void expire(TargetController& controller, std::uint32_t id, Time firstMiss)
{
    for (auto miss = 0; miss < 3; ++miss)
    {
        const auto time = firstMiss + Time(1000 * miss);
        const auto request = controller.handleDetection(result(BeamType::TRACKING, id, time, false));
        if (miss < 2)
        {
            ASSERT_TRUE(request);
            EXPECT_EQ(request->requestId, id);
            EXPECT_EQ(request->transmitTime, time + 1000ms);
        }
        else
        {
            EXPECT_FALSE(request);
        }
    }
}

TEST(TrackReportingLimitTest, HandlesZeroNinetyNineOneHundredAndOneHundredOneTrackingTargets)
{
    auto controller = TargetController(
        TargetConfig(
            AssociationConfig(1000_mps, 30_mps, 11.345),
            MeasurementConfig(7_m),
            TrackingConfig(10_mps2, 0_mps2),
            TrackManagementConfig(100ms, 1000ms, 3, 100)
        )
    );
    addCandidates(controller, 103);
    // 처음과 마지막 트랙을 잠정 상태로 남겨 보고 한도에 포함되는지 확인한다.
    expectReportedIds(controller, 0, 2);
    confirm(controller, 2, 100);
    expectReportedIds(controller, 99, 2);
    confirm(controller, 101, 101);
    expectReportedIds(controller, 100, 2);
    confirm(controller, 102, 102);
    expectReportedIds(controller, 100, 2);
    // 연속 조회해도 같은 트랙을 같은 순서로 보고해야 한다.
    expectReportedIds(controller, 100, 2);
}

TEST(TrackReportingLimitTest, UnreportedTargetCanUpdateAndExpireWithoutAffectingTheFirstHundred)
{
    auto controller = TargetController(
        TargetConfig(
            AssociationConfig(1000_mps, 30_mps, 11.345),
            MeasurementConfig(7_m),
            TrackingConfig(10_mps2, 0_mps2),
            TrackManagementConfig(100ms, 1000ms, 3, 100)
        )
    );
    addCandidates(controller, 101);
    confirm(controller, 1, 101);
    expectReportedIds(controller, 100, 1);

    const auto updated = controller.handleDetection(result(BeamType::TRACKING, 101, 1100ms));
    ASSERT_TRUE(updated);
    EXPECT_EQ(updated->requestId, 101u);
    EXPECT_EQ(updated->transmitTime, 2100ms);
    expectReportedIds(controller, 100, 1);
    expire(controller, 101, 2100ms);
    expectReportedIds(controller, 100, 1);
    // 다른 트랙들이 1100ms 추적 결과의 응답 대기 한도(4100ms)를 넘지 않는 시각으로 확인한다.
    EXPECT_FALSE(controller.handleDetection(result(BeamType::TRACKING, 101, 4100ms)));

    // 보고 중인 트랙 하나를 만료시켜도 보고 수가 99개라면, 101번 트랙은 저장소에서도 제거된 것이다.
    expire(controller, 1, 1100ms);
    expectReportedIds(controller, 99, 2);
}

TEST(TrackReportingLimitTest, RevealsTheNextStoredTrackingTargetWhenAReportedTargetExpires)
{
    auto controller = TargetController(
        TargetConfig(
            AssociationConfig(1000_mps, 30_mps, 11.345),
            MeasurementConfig(7_m),
            TrackingConfig(10_mps2, 0_mps2),
            TrackManagementConfig(100ms, 1000ms, 3, 100)
        )
    );
    addCandidates(controller, 101);
    confirm(controller, 1, 101);
    expectReportedIds(controller, 100, 1);
    expire(controller, 1, 1100ms);
    expectReportedIds(controller, 100, 2);
    expectReportedIds(controller, 100, 2);
}

TEST(TrackReportingLimitTest, UsesInjectedLimitWithoutLimitingTentativeOrHiddenTrackingTargets)
{
    const auto config = TargetConfig(
        AssociationConfig(1000_mps, 30_mps, 11.345),
        MeasurementConfig(7_m),
        TrackingConfig(10_mps2, 0_mps2),
        TrackManagementConfig(100ms, 1000ms, 3, 2)
    );
    auto controller = TargetController(config);
    addCandidates(controller, 5);
    confirm(controller, 2, 5);
    expectReportedIds(controller, 2, 2);
    expectReportedIds(controller, 2, 2);

    const auto hiddenRequest = controller.handleDetection(result(BeamType::TRACKING, 5, 1100ms));
    ASSERT_TRUE(hiddenRequest);
    EXPECT_EQ(hiddenRequest->requestId, 5u);
    EXPECT_EQ(hiddenRequest->transmitTime, 2100ms);
    expectReportedIds(controller, 2, 2);
    expire(controller, 2, 1100ms);
    expectReportedIds(controller, 2, 3);
}

TEST(TrackReportingLimitTest, ValidatesConfiguredCountAgainstTheMessageMaximum)
{
    EXPECT_EQ(TrackManagementConfig::MAXIMUM_REPORTED_TRACK_COUNT, 100u);
    for (auto count : {std::size_t(1), std::size_t(100)})
    {
        const auto config = TargetConfig(
            AssociationConfig(1000_mps, 30_mps, 11.345),
            MeasurementConfig(7_m),
            TrackingConfig(10_mps2, 0_mps2),
            TrackManagementConfig(100ms, 1000ms, 3, count)
        );
        EXPECT_EQ(config.trackManagement.maxReportedTrackCount(), count);
    }
    for (auto count : {std::size_t(0), std::size_t(101), std::numeric_limits<std::size_t>::max()})
    {
        EXPECT_THROW(
            TargetConfig(
                AssociationConfig(1000_mps, 30_mps, 11.345),
                MeasurementConfig(7_m),
                TrackingConfig(10_mps2, 0_mps2),
                TrackManagementConfig(100ms, 1000ms, 3, count)
            ),
            std::invalid_argument
        );
    }
}

} // namespace
} // namespace target
