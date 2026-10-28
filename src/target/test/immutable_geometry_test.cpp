#include "calculation/tracking/constant_velocity_algorithm.hpp"
#include "repository/target_repository.hpp"
#include <target/config/target_config.hpp>
#include "target/controller/target_controller.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <memory>

using namespace math::literals;
using namespace std::chrono_literals;

namespace
{
math::SphericalPosition position(math::Length range)
{
    return math::SphericalPosition(range, math::SphericalDirection(math::Azimuth(0_deg), math::Elevation(0_deg)));
}
} // namespace

TEST(ImmutableGeometryTest, ReplacesTrackingStateAndRemovesTargetsInOrder)
{
    const auto algorithm = std::make_shared<target::ConstantVelocityAlgorithm>();
    target::TargetRepository repository;
    const auto firstId = repository.add(target::Target(position(100_m), 1_mps, 1.0f, 0ms, algorithm));
    const auto middleId = repository.add(target::Target(position(200_m), 1_mps, 1.0f, 0ms, algorithm));
    const auto lastId = repository.add(target::Target(position(300_m), 1_mps, 1.0f, 0ms, algorithm));

    auto* tracked = repository.find(lastId);
    ASSERT_TRUE(tracked->confirm(target::Target(position(301_m), 1_mps, 1.0f, 1000ms, algorithm)));
    ASSERT_TRUE(tracked->update(target::Target(position(302_m), 1_mps, 1.0f, 2000ms, algorithm)));
    EXPECT_EQ(tracked->trackingState()->position().x, 0_m);
    EXPECT_EQ(tracked->trackingState()->position().y, 302_m);
    EXPECT_EQ(tracked->trackingState()->velocity().x, 0_mps);
    EXPECT_EQ(tracked->trackingState()->velocity().y, 1_mps);

    EXPECT_FALSE(repository.remove(0));
    ASSERT_TRUE(repository.remove(middleId));
    ASSERT_EQ(repository.targets().size(), 2u);
    EXPECT_EQ(repository.targets()[0].id(), firstId);
    EXPECT_EQ(repository.targets()[1].id(), lastId);
    EXPECT_EQ(repository.find(middleId), nullptr);
    tracked = repository.find(lastId);
    ASSERT_TRUE(tracked->trackingState());
    EXPECT_EQ(tracked->trackingState()->position().y, 302_m);
    ASSERT_TRUE(tracked->update(target::Target(position(303_m), 1_mps, 1.0f, 3000ms, algorithm)));
    EXPECT_EQ(tracked->trackingState()->position().y, 303_m);

    EXPECT_TRUE(repository.remove(firstId));
    EXPECT_TRUE(repository.remove(lastId));
    EXPECT_TRUE(repository.targets().empty());
    EXPECT_FALSE(repository.remove(lastId));
}

TEST(ImmutableGeometryTest, ReportsStationaryAndMovingTargetSpeed)
{
    using BeamType = beam::BeamType;
    for (double displacement : {0.0, 1.0})
    {
        target::TargetController controller(target::TargetConfig(
            1000_mps, 30_mps, 7_m, 11.345, 100ms, 1000ms, 3, 10.0, 100));
        const auto confirmation = controller.handleDetection(target::DetectionEvent(
            BeamType::SEARCH, 1, 0ms, {{100_m, math::Azimuth(0_deg), math::Elevation(0_deg), 0_mps, 1.0f}}));
        ASSERT_TRUE(confirmation.has_value());
        const auto tracking = controller.handleDetection(target::DetectionEvent(
            BeamType::CONFIRMATION,
            confirmation->requestId,
            1000ms,
            {{math::Length::fromMeters(100.0 + displacement), math::Azimuth(0_deg), math::Elevation(0_deg), 0_mps, 1.0f}}));
        ASSERT_TRUE(tracking.has_value());
        const auto events = controller.nextTrackEvents();
        ASSERT_EQ(events.size(), 1u);
        EXPECT_DOUBLE_EQ(events[0].velocity.mps(), displacement);
        EXPECT_DOUBLE_EQ(events[0].latitude.deg(), 0.0);
        EXPECT_DOUBLE_EQ(events[0].longitude.deg(), 0.0);
        EXPECT_DOUBLE_EQ(events[0].heading.deg(), 0.0);
        EXPECT_DOUBLE_EQ(events[0].flightPathAngle.deg(), 0.0);
    }
}
