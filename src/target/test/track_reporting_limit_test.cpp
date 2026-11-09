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
    // Register all equally timed searches before confirmations, avoiding any
    // motion allowance against a newer Tracking observation during test setup.
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
    for (std::uint32_t id = first; id <= last; ++id)
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
    std::uint16_t first)
{
    const auto events = controller.nextTrackEvents();
    ASSERT_EQ(events.size(), count);
    for (std::size_t index = 0; index < count; ++index)
    {
        const auto id = static_cast<std::uint16_t>(static_cast<std::size_t>(first) + index);
        EXPECT_EQ(events[index].id, id);
        EXPECT_EQ(events[index].state, TrackEvent::TrackState::TRACKING);
        EXPECT_EQ(events[index].timestamp, 100ms);
        EXPECT_NEAR(events[index].slantRange.m(), range(id).m(), 1e-9);
    }
}

void expire(TargetController& controller, std::uint32_t id, Time firstMiss)
{
    for (int miss = 0; miss < 3; ++miss)
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
    TargetController controller(TargetConfig(1000_mps, 30_mps, 7_m, 11.345, 100ms, 1000ms, 3, 10_mps2, 100, 0_mps2));
    addCandidates(controller, 103);
    // The first and last repository entries stay Tentative and never consume
    // a reporting slot. There are initially zero Tracking entries.
    expectReportedIds(controller, 0, 2);
    confirm(controller, 2, 100);
    expectReportedIds(controller, 99, 2);
    confirm(controller, 101, 101);
    expectReportedIds(controller, 100, 2);
    confirm(controller, 102, 102);
    expectReportedIds(controller, 100, 2);
    // Reading neither consumes entries nor rotates which hundred are reported.
    expectReportedIds(controller, 100, 2);
}

TEST(TrackReportingLimitTest, UnreportedTargetCanUpdateAndExpireWithoutAffectingTheFirstHundred)
{
    TargetController controller(TargetConfig(1000_mps, 30_mps, 7_m, 11.345, 100ms, 1000ms, 3, 10_mps2, 100, 0_mps2));
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
    EXPECT_FALSE(controller.handleDetection(result(BeamType::TRACKING, 101, 5100ms)));

    // Removing one visible target leaves 99: the hidden target was actually
    // removed after three misses rather than merely omitted from reporting.
    expire(controller, 1, 1100ms);
    expectReportedIds(controller, 99, 2);
}

TEST(TrackReportingLimitTest, RevealsTheNextStoredTrackingTargetWhenAReportedTargetExpires)
{
    TargetController controller(TargetConfig(1000_mps, 30_mps, 7_m, 11.345, 100ms, 1000ms, 3, 10_mps2, 100, 0_mps2));
    addCandidates(controller, 101);
    confirm(controller, 1, 101);
    expectReportedIds(controller, 100, 1);
    expire(controller, 1, 1100ms);
    expectReportedIds(controller, 100, 2);
    expectReportedIds(controller, 100, 2);
}

TEST(TrackReportingLimitTest, UsesInjectedLimitWithoutLimitingTentativeOrHiddenTrackingTargets)
{
    const TargetConfig config(1000_mps, 30_mps, 7_m, 11.345, 100ms, 1000ms, 3, 10_mps2, 2, 0_mps2);
    TargetController controller(config);
    addCandidates(controller, 5);
    // ID 1 stays Tentative; IDs 2..5 all become Tracking despite the reporting limit.
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
    EXPECT_EQ(TargetConfig::MAXIMUM_REPORTED_TRACK_COUNT, 100u);
    for (std::size_t count : {std::size_t{1}, std::size_t{100}})
    {
        const TargetConfig config(1000_mps, 30_mps, 7_m, 11.345,
            100ms, 1000ms, 3, 10_mps2, count, 0_mps2);
        EXPECT_EQ(config.maxReportedTrackCount(), count);
    }
    for (std::size_t count : {std::size_t{0}, std::size_t{101},
             std::numeric_limits<std::size_t>::max()})
    {
        EXPECT_THROW(TargetConfig(1000_mps, 30_mps, 7_m, 11.345,
            100ms, 1000ms, 3, 10_mps2, count, 0_mps2), std::invalid_argument);
    }
}

} // namespace
} // namespace target
