#include "domain/tracking/constant_acceleration_algorithm.hpp"
#include "domain/association/track_selector.hpp"
#include "domain/association/tentative_selector.hpp"

#include <target/config/target_config.hpp>
#include <target/controller/target_controller.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <type_traits>

namespace target
{
namespace
{

using namespace math::literals;
using namespace std::chrono_literals;

static_assert(!std::is_default_constructible_v<TargetConfig>);
static_assert(std::is_copy_assignable_v<TargetConfig>);
static_assert(!std::is_constructible_v<TrackSelector, TargetConfig>);
static_assert(std::is_constructible_v<TrackSelector, AssociationConfig, std::unique_ptr<IAssociationAlgorithm>>);
static_assert(std::is_constructible_v<TentativeSelector, AssociationConfig, std::unique_ptr<IAssociationAlgorithm>>);
static_assert(!std::is_constructible_v<ConstantAccelerationAlgorithm, TargetConfig>);
static_assert(std::is_constructible_v<ConstantAccelerationAlgorithm, TrackingConfig, MeasurementConfig>);

DetectionEvent detection(beam::BeamType type, std::uint32_t id, std::chrono::milliseconds time, math::Length range)
{
    return DetectionEvent(
        type,
        id,
        time,
        {DetectionEvent::Plot(math::Distance(range), math::CircularAngle(0_deg), math::TiltAngle(0_deg), 0_mps, -10.0f)}
    );
}

TEST(TargetConfigTest, BuilderRequiresEveryComponent)
{
    for (unsigned int mask = 0; mask < 31; ++mask)
    {
        SCOPED_TRACE(mask);
        TargetConfig::Builder builder;
        if (mask & 1)
        {
            builder.associationConfig(AssociationConfig(1000_mps, 30_mps, 11.345));
        }
        if (mask & 2)
        {
            builder.measurementConfig(MeasurementConfig(7_m));
        }
        if (mask & 4)
        {
            builder.trackingConfig(TrackingConfig(10_mps2, 0_mps2));
        }
        if (mask & 8)
        {
            builder.trackLifecycleConfig(TrackLifecycleConfig(100ms, 1000ms, 3));
        }
        if (mask & 16)
        {
            builder.trackReportingConfig(TrackReportingConfig(100));
        }
        EXPECT_THROW(builder.build(), std::logic_error);
    }
}

TEST(AssociationConfigTest, BuilderRequiresEveryValue)
{
    for (unsigned int mask = 0; mask < 7; ++mask)
    {
        SCOPED_TRACE(mask);
        AssociationConfig::Builder builder;
        if (mask & 1)
        {
            builder.maxAssociationSpeed(0_mps);
        }
        if (mask & 2)
        {
            builder.maxInitialDopplerDifference(0_mps);
        }
        if (mask & 4)
        {
            builder.positionNdsThreshold(0.0);
        }
        EXPECT_THROW(builder.build(), std::logic_error);
    }
}

TEST(MeasurementConfigTest, BuilderRequiresStandardDeviationAndRejectsUnusableVariance)
{
    EXPECT_THROW(MeasurementConfig::Builder().build(), std::logic_error);
    for (auto sigma : {0.0, -1.0, 1e-300, 1.2e-162, 1e308})
    {
        SCOPED_TRACE(sigma);
        EXPECT_THROW(
            MeasurementConfig::Builder()
                .standardDeviation(math::Length::fromMeters(sigma))
                .build(),
            std::invalid_argument
        );
    }
}

TEST(TrackingConfigTest, BuilderRequiresBothValuesAndValidatesNoise)
{
    EXPECT_THROW(TrackingConfig::Builder().build(), std::logic_error);
    EXPECT_THROW(
        TrackingConfig::Builder()
            .initialAccelerationStandardDeviation(10_mps2)
            .build(),
        std::logic_error
    );
    EXPECT_THROW(
        TrackingConfig::Builder()
            .processAccelerationStandardDeviation(0_mps2)
            .build(),
        std::logic_error
    );
    EXPECT_THROW(
        TrackingConfig::Builder()
            .initialAccelerationStandardDeviation(0_mps2)
            .processAccelerationStandardDeviation(0_mps2)
            .build(),
        std::invalid_argument
    );
    EXPECT_THROW(
        TrackingConfig::Builder()
            .initialAccelerationStandardDeviation(10_mps2)
            .processAccelerationStandardDeviation(1e-300_mps2)
            .build(),
        std::invalid_argument
    );
}

TEST(TrackLifecycleConfigTest, BuilderRequiresEveryValue)
{
    for (unsigned int mask = 0; mask < 7; ++mask)
    {
        SCOPED_TRACE(mask);
        TrackLifecycleConfig::Builder builder;
        if (mask & 1)
        {
            builder.confirmationDelay(100ms);
        }
        if (mask & 2)
        {
            builder.trackingInterval(1000ms);
        }
        if (mask & 4)
        {
            builder.maxConsecutiveTrackingMisses(3);
        }
        EXPECT_THROW(builder.build(), std::logic_error);
    }
}

TEST(TrackLifecycleConfigTest, ValidatesDurationsAndMissLimitAndDerivesResponseTimeout)
{
    EXPECT_THROW(TrackLifecycleConfig(0ms, 1000ms, 3), std::invalid_argument);
    EXPECT_THROW(TrackLifecycleConfig(-1ms, 1000ms, 3), std::invalid_argument);
    EXPECT_THROW(TrackLifecycleConfig(100ms, 0ms, 3), std::invalid_argument);
    EXPECT_THROW(TrackLifecycleConfig(100ms, -1ms, 3), std::invalid_argument);
    EXPECT_THROW(TrackLifecycleConfig(100ms, 1000ms, 0), std::invalid_argument);
    const auto config = TrackLifecycleConfig::Builder()
        .confirmationDelay(250ms)
        .trackingInterval(400ms)
        .maxConsecutiveTrackingMisses(2)
        .build();
    EXPECT_EQ(config.confirmationDelay(), 250ms);
    EXPECT_EQ(config.trackingInterval(), 400ms);
    EXPECT_EQ(config.maxConsecutiveTrackingMisses(), 2u);
    EXPECT_EQ(config.responseTimeout(), 800ms);
}

TEST(TrackReportingConfigTest, BuilderRequiresCountAndValidatesReportLimit)
{
    EXPECT_THROW(TrackReportingConfig::Builder().build(), std::logic_error);
    EXPECT_THROW(TrackReportingConfig::Builder().maxReportedTrackCount(0).build(), std::invalid_argument);
    EXPECT_THROW(
        TrackReportingConfig::Builder()
            .maxReportedTrackCount(101)
            .build(),
        std::invalid_argument
    );
    EXPECT_EQ(TrackReportingConfig::Builder().maxReportedTrackCount(1).build().maxReportedTrackCount(), 1u);
    EXPECT_EQ(TrackReportingConfig::Builder().maxReportedTrackCount(100).build().maxReportedTrackCount(), 100u);
}

TEST(TargetConfigTest, BuildersAcceptExplicitZeroValuesAndFeedTheController)
{
    const auto config = TargetConfig::Builder()
        .associationConfig(
            AssociationConfig::Builder()
                .maxAssociationSpeed(0_mps)
                .maxInitialDopplerDifference(0_mps)
                .positionNdsThreshold(0.0)
                .build()
        )
        .measurementConfig(MeasurementConfig::Builder().standardDeviation(7_m).build())
        .trackingConfig(
            TrackingConfig::Builder()
                .initialAccelerationStandardDeviation(10_mps2)
                .processAccelerationStandardDeviation(0_mps2)
                .build()
        )
        .trackLifecycleConfig(
            TrackLifecycleConfig::Builder()
                .confirmationDelay(100ms)
                .trackingInterval(1000ms)
                .maxConsecutiveTrackingMisses(3)
                .build()
        )
        .trackReportingConfig(TrackReportingConfig::Builder().maxReportedTrackCount(1).build())
        .build();
    auto controller = TargetController(config);
    controller.setEnuGeoTransform(pose::SetEnuGeoTransformCommand(pose::EnuGeoTransform(pose::GeoPosition(math::TiltAngle(0_deg), math::CircularAngle(0_deg), 100_m))));
    const auto confirmation = controller.handleDetection(detection(beam::BeamType::SEARCH, 0, 0ms, 1000_m));
    ASSERT_TRUE(confirmation);
    const auto tracking = controller.handleDetection(
        detection(
            beam::BeamType::CONFIRMATION,
            confirmation->requestId,
            confirmation->transmitTime,
            1000_m
        )
    );
    ASSERT_TRUE(tracking);
    EXPECT_EQ(tracking->transmitTime, 1100ms);
    ASSERT_EQ(controller.getTrackEvents().size(), 1u);
    EXPECT_EQ(controller.getTrackEvents().front().id, confirmation->requestId);
}

TEST(TargetConfigTest, ControllerKeepsSnapshotsWhenComponentsAreReplaced)
{
    auto config = TargetConfig(
        AssociationConfig(1000_mps, 30_mps, 11.345),
        MeasurementConfig(7_m),
        TrackingConfig(10_mps2, 0_mps2),
        TrackLifecycleConfig(100ms, 1000ms, 3),
        TrackReportingConfig(100)
    );
    auto controller = TargetController(config);
    controller.setEnuGeoTransform(pose::SetEnuGeoTransformCommand(pose::EnuGeoTransform(pose::GeoPosition(math::TiltAngle(0_deg), math::CircularAngle(0_deg), 100_m))));
    config.association = AssociationConfig(0_mps, 0_mps, 0.0);
    config.measurement = MeasurementConfig(1_m);
    config.tracking = TrackingConfig(1_mps2, 100_mps2);
    config.trackLifecycle = TrackLifecycleConfig(1ms, 1ms, 1);
    config.trackReporting = TrackReportingConfig(1);

    const auto confirmation = controller.handleDetection(detection(beam::BeamType::SEARCH, 0, 0ms, 1000_m));
    ASSERT_TRUE(confirmation);
    EXPECT_EQ(confirmation->transmitTime, 100ms);
    const auto tracking = controller.handleDetection(
        detection(
            beam::BeamType::CONFIRMATION,
            confirmation->requestId,
            100ms,
            1001_m
        )
    );
    ASSERT_TRUE(tracking);
    EXPECT_EQ(tracking->transmitTime, 1100ms);
    const auto retry = controller.handleDetection(
        DetectionEvent(
            beam::BeamType::TRACKING,
            confirmation->requestId,
            1100ms,
            {}
        )
    );
    ASSERT_TRUE(retry);
    EXPECT_EQ(retry->transmitTime, 2100ms);
    EXPECT_EQ(controller.getTrackEvents().size(), 1u);
}

} // namespace
} // namespace target
