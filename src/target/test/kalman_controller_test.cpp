#include <target/config/target_config.hpp>
#include <target/controller/target_controller.hpp>

#include <math/geometry/cartesian_displacement.hpp>
#include <math/geometry/cartesian_position.hpp>
#include <math/geometry/spherical_position.hpp>

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>
#include <initializer_list>
#include <optional>
#include <utility>
#include <vector>

namespace target
{
namespace
{

using namespace math::literals;
using namespace std::chrono_literals;
using Time = std::chrono::milliseconds;
using BeamType = beam::BeamCommand::BeamType;

math::CartesianPosition radial(math::Length range)
{
    return math::CartesianPosition(0_m, range, 0_m);
}

DetectionEvent result(
    BeamType type,
    std::uint32_t id,
    Time time,
    std::initializer_list<math::CartesianPosition> positions)
{
    std::vector<DetectionEvent::Plot> plots;
    for (const auto& position : positions)
    {
        const auto spherical = position.toSpherical();
        plots.emplace_back(spherical.radius, spherical.direction.azimuth,
            spherical.direction.elevation, 0_mps, -10.0f);
    }
    return DetectionEvent(type, id, time, std::move(plots));
}

std::optional<beam::BeamRequest> beginTracking(
    TargetController& controller,
    const math::CartesianPosition& first,
    const math::CartesianPosition& second)
{
    const auto confirmation = controller.handleDetection(result(BeamType::SEARCH, 37, 0ms, {first}));
    if (!confirmation)
    {
        ADD_FAILURE() << "A new search observation must produce a confirmation request";
        return std::nullopt;
    }
    EXPECT_EQ(confirmation->beamType, beam::BeamRequest::BeamType::CONFIRMATION);
    EXPECT_EQ(confirmation->requestId, 1u);
    EXPECT_EQ(confirmation->transmitTime, 100ms);
    return controller.handleDetection(result(BeamType::CONFIRMATION,
        confirmation->requestId, confirmation->transmitTime, {second}));
}

math::CartesianPosition consolePosition(const TrackEvent& event)
{
    return math::SphericalPosition(event.slantRange,
        math::SphericalDirection(event.azimuth, event.elevation)).toCartesian();
}

math::CartesianPosition trajectory(double seconds)
{
    // True antenna-frame motion: p0=(3000,10000,1000), v0=(80,-40,10), a=(6,3,-2).
    return math::CartesianPosition(
        math::Length::fromMeters(3000.0 + 80.0 * seconds + 3.0 * seconds * seconds),
        math::Length::fromMeters(10000.0 - 40.0 * seconds + 1.5 * seconds * seconds),
        math::Length::fromMeters(1000.0 + 10.0 * seconds - seconds * seconds));
}

TEST(KalmanControllerTest, PreservesGeneratedIdAndReportsFilteredCoordinatesInsteadOfTheRawPlot)
{
    TargetController controller(TargetConfig{});
    const auto tracking = beginTracking(controller, radial(1000_m), radial(1000_m));
    ASSERT_TRUE(tracking);
    EXPECT_EQ(tracking->beamType, beam::BeamRequest::BeamType::TRACKING);
    EXPECT_EQ(tracking->requestId, 1u);
    EXPECT_EQ(tracking->transmitTime, 1100ms);
    EXPECT_NEAR(tracking->azimuth_ant.deg(), 0.0, 1e-12);
    EXPECT_NEAR(tracking->elevation_ant.deg(), 0.0, 1e-12);

    const auto next = controller.handleDetection(result(BeamType::TRACKING,
        tracking->requestId, tracking->transmitTime, {radial(1100_m)}));
    ASSERT_TRUE(next);
    EXPECT_EQ(next->requestId, tracking->requestId);
    EXPECT_EQ(next->transmitTime, 2100ms);
    const auto events = controller.nextTrackEvents();
    ASSERT_EQ(events.size(), 1u);
    EXPECT_EQ(events[0].id, tracking->requestId);
    EXPECT_EQ(events[0].timestamp, 1100ms);
    const double filteredRange = events[0].slantRange.m();
    EXPECT_GT(filteredRange, 1000.0);
    EXPECT_LT(filteredRange, 1099.99);
    EXPECT_GT(events[0].velocity.mps(), 0.0);
    EXPECT_NEAR(consolePosition(events[0]).y.m(), filteredRange, 1e-9);
}

TEST(KalmanControllerTest, SelectsTheBestPlotInsideThePredictedCovarianceGateAndIgnoresExtraPlots)
{
    TargetController controller(TargetConfig{});
    const auto tracking = beginTracking(controller, radial(1000_m), radial(1000_m));
    ASSERT_TRUE(tracking);
    // Both 50 m and 200 m residuals fit the uncertain two-point forecast.
    // A fixed 98 I covariance would reject both, so this also checks propagation into association.
    const auto next = controller.handleDetection(result(BeamType::TRACKING,
        tracking->requestId, 1100ms, {radial(1200_m), radial(1050_m), radial(5000_m)}));
    ASSERT_TRUE(next);
    EXPECT_EQ(next->requestId, tracking->requestId);
    const auto events = controller.nextTrackEvents();
    ASSERT_EQ(events.size(), 1u);
    EXPECT_GT(events[0].slantRange.m(), 1040.0);
    EXPECT_LT(events[0].slantRange.m(), 1050.0);
    EXPECT_EQ(events[0].timestamp, 1100ms);

    const auto newCandidate = controller.handleDetection(result(BeamType::SEARCH,
        38, 1200ms, {radial(5000_m)}));
    ASSERT_TRUE(newCandidate);
    EXPECT_EQ(newCandidate->requestId, 2u);
    EXPECT_EQ(controller.nextTrackEvents().size(), 1u);
}

TEST(KalmanControllerTest, LearnsThreeAxisAccelerationFromNoisyPlotsAndKeepsForecastingDuringMisses)
{
    TargetController controller(TargetConfig{});
    const auto tracking = beginTracking(controller, trajectory(0.0), trajectory(0.1));
    ASSERT_TRUE(tracking);
    const std::array<std::array<double, 3>, 12> noiseMultipliers{{
        {{1.0, -0.5, 0.7}}, {{-0.8, 0.9, -1.0}}, {{0.5, -1.0, 0.4}},
        {{-1.0, 0.3, -0.6}}, {{0.7, 0.6, 1.0}}, {{-0.4, -0.8, -0.3}},
        {{0.9, 0.2, -0.9}}, {{-0.6, 0.8, 0.5}}, {{0.2, -0.7, -0.4}},
        {{-0.7, 0.4, 0.8}}, {{0.4, -0.2, -0.7}}, {{-0.3, 0.7, 0.2}}
    }};
    double measuredSquaredError = 0.0;
    double filteredSquaredError = 0.0;
    Time observationTime = 100ms;
    for (const auto& noise : noiseMultipliers)
    {
        observationTime += 1000ms;
        const double seconds = std::chrono::duration<double>(observationTime).count();
        const auto truth = trajectory(seconds);
        // Fixed Cartesian errors scaled by the configured 7 m per-axis measurement noise.
        const math::CartesianPosition measured(
            math::Length::fromMeters(truth.x.m() + 7.0 * noise[0]),
            math::Length::fromMeters(truth.y.m() + 7.0 * noise[1]),
            math::Length::fromMeters(truth.z.m() + 7.0 * noise[2]));
        const auto next = controller.handleDetection(result(BeamType::TRACKING,
            tracking->requestId, observationTime, {measured}));
        ASSERT_TRUE(next) << "Update at " << observationTime.count() << " ms";
        EXPECT_EQ(next->requestId, tracking->requestId);
        EXPECT_EQ(next->transmitTime, observationTime + 1000ms);
        const auto events = controller.nextTrackEvents();
        ASSERT_EQ(events.size(), 1u);
        EXPECT_EQ(events[0].timestamp, observationTime);
        const double measuredError = (measured - truth).distance().m();
        const double filteredError = (consolePosition(events[0]) - truth).distance().m();
        measuredSquaredError += measuredError * measuredError;
        filteredSquaredError += filteredError * filteredError;
    }
    EXPECT_LT(filteredSquaredError, 0.8 * measuredSquaredError);
    const auto finalEvents = controller.nextTrackEvents();
    ASSERT_EQ(finalEvents.size(), 1u);
    EXPECT_LT((consolePosition(finalEvents[0]) - trajectory(12.1)).distance().m(), 7.0);
    const double trueSpeed = std::hypot(80.0 + 6.0 * 12.1, -40.0 + 3.0 * 12.1,
        10.0 - 2.0 * 12.1);
    EXPECT_NEAR(finalEvents[0].velocity.mps(), trueSpeed, 3.0);

    for (int miss = 1; miss <= 3; ++miss)
    {
        const auto resultTime = observationTime + std::chrono::milliseconds(1000 * miss);
        const auto retry = controller.handleDetection(result(BeamType::TRACKING,
            tracking->requestId, resultTime, {}));
        if (miss < 3)
        {
            ASSERT_TRUE(retry);
            EXPECT_EQ(retry->requestId, tracking->requestId);
            EXPECT_EQ(retry->transmitTime, resultTime + 1000ms);
            const auto truth = trajectory(std::chrono::duration<double>(retry->transmitTime).count()).toSpherical();
            EXPECT_NEAR(retry->azimuth_ant.deg(), truth.direction.azimuth.deg(), 0.05);
            EXPECT_NEAR(retry->elevation_ant.deg(), truth.direction.elevation.deg(), 0.05);
            const auto events = controller.nextTrackEvents();
            ASSERT_EQ(events.size(), 1u);
            EXPECT_EQ(events[0].timestamp, observationTime);
            EXPECT_EQ(events[0].slantRange, finalEvents[0].slantRange);
        }
        else
        {
            EXPECT_FALSE(retry);
            EXPECT_TRUE(controller.nextTrackEvents().empty());
        }
    }
}

TEST(KalmanControllerTest, InjectedAccelerationUncertaintyControlsRecoveryGateAndSuccessResetsMisses)
{
    for (double accelerationSigma : {1.0, 100.0})
    {
        SCOPED_TRACE(accelerationSigma);
        const TargetConfig config(1000_mps, 30_mps, 7_m, 11.345,
            100ms, 1000ms, 3, accelerationSigma);
        TargetController controller(config);
        const auto tracking = beginTracking(controller, radial(1000_m), radial(1000_m));
        ASSERT_TRUE(tracking);
        ASSERT_TRUE(controller.handleDetection(result(BeamType::TRACKING,
            tracking->requestId, 1100ms, {})));
        ASSERT_TRUE(controller.handleDetection(result(BeamType::TRACKING,
            tracking->requestId, 2100ms, {})));
        // After 3 s, a 1400 m residual has NDS about 21.5 for sigma_a=1,
        // but 6.4 for sigma_a=100; only the latter passes the shared 99% gate.
        const auto recovered = controller.handleDetection(result(BeamType::TRACKING,
            tracking->requestId, 3100ms, {radial(2400_m)}));
        const bool broadPrior = accelerationSigma > 1.0;
        EXPECT_EQ(recovered.has_value(), broadPrior);
        if (!broadPrior)
        {
            EXPECT_TRUE(controller.nextTrackEvents().empty());
            continue;
        }
        EXPECT_EQ(recovered->requestId, tracking->requestId);
        const auto events = controller.nextTrackEvents();
        ASSERT_EQ(events.size(), 1u);
        EXPECT_EQ(events[0].timestamp, 3100ms);
        ASSERT_TRUE(controller.handleDetection(result(BeamType::TRACKING,
            tracking->requestId, 4100ms, {})));
        ASSERT_TRUE(controller.handleDetection(result(BeamType::TRACKING,
            tracking->requestId, 5100ms, {})));
        EXPECT_FALSE(controller.handleDetection(result(BeamType::TRACKING,
            tracking->requestId, 6100ms, {})));
        EXPECT_TRUE(controller.nextTrackEvents().empty());
    }
}

} // namespace
} // namespace target
