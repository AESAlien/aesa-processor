#include "calculation/association/detection_matcher.hpp"
#include "calculation/tracking/constant_velocity_algorithm.hpp"
#include "repository/target_repository.hpp"
#include "selector/track_selector.hpp"

#include <target/config/target_config.hpp>
#include <target/controller/target_controller.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <memory>
#include <optional>
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

math::SphericalPosition position(math::Length range)
{
    return math::SphericalPosition(range, math::SphericalDirection(0_deg, 0_deg));
}

DetectionEvent result(
    BeamType type,
    std::uint32_t id,
    Time time,
    std::initializer_list<math::Length> ranges)
{
    std::vector<DetectionEvent::Plot> plots;
    for (const auto range : ranges)
    {
        plots.emplace_back(range, 0_deg, 0_deg, 0_mps, -10.0f);
    }
    return DetectionEvent(type, id, time, std::move(plots));
}

class NoPredictionAlgorithm final : public ITrackingAlgorithm
{
public:
    std::optional<TargetTrackingState> initialize(const Target& first, const Target& second) const override
    {
        return _algorithm.initialize(first, second);
    }

    std::optional<TargetTrackingState> update(const Target& current, const Target& incoming) const override
    {
        return _algorithm.update(current, incoming);
    }

    std::optional<TargetPrediction> predict(const Target&, Time) const override
    {
        return std::nullopt;
    }

private:
    ConstantVelocityAlgorithm _algorithm;
};

class CovarianceAlgorithm final : public ITrackingAlgorithm
{
public:
    explicit CovarianceAlgorithm(math::SquareMatrix positionCovariance, double growthPerSecond = 0.0)
        : _positionCovariance(std::move(positionCovariance))
        , _growthPerSecond(growthPerSecond)
    {
    }

    std::optional<TargetTrackingState> initialize(const Target& first, const Target& second) const override
    {
        return withCovariance(_algorithm.initialize(first, second));
    }

    std::optional<TargetTrackingState> update(const Target& current, const Target& incoming) const override
    {
        return withCovariance(_algorithm.update(current, incoming));
    }

    std::optional<TargetPrediction> predict(const Target& current, Time time) const override
    {
        const auto prediction = _algorithm.predict(current, time);
        if (!prediction)
        {
            return std::nullopt;
        }
        const double seconds = std::chrono::duration<double>(time - current.lastObservationTime()).count();
        return TargetPrediction(time, prediction->sphericalPosition,
            _positionCovariance * (1.0 + _growthPerSecond * seconds));
    }

private:
    std::optional<TargetTrackingState> withCovariance(
        const std::optional<TargetTrackingState>& state) const
    {
        if (!state)
        {
            return std::nullopt;
        }
        math::SquareMatrix covariance(TargetTrackingState::STATE_DIMENSION);
        for (std::size_t row = 0; row < TargetPrediction::POSITION_DIMENSION; ++row)
        {
            for (std::size_t column = 0; column < TargetPrediction::POSITION_DIMENSION; ++column)
            {
                covariance(row, column) = _positionCovariance(row, column);
            }
        }
        return TargetTrackingState(state->sphericalPosition, state->dopplerVelocity,
            state->power_db, state->timestamp, state->position, state->velocity,
            state->acceleration, std::move(covariance));
    }

    const math::SquareMatrix _positionCovariance;
    const double _growthPerSecond;
    ConstantVelocityAlgorithm _algorithm;
};

class TrackingAssociationTest : public ::testing::Test
{
protected:
    Target observation(math::Length range, Time time) const
    {
        return Target(position(range), 0_mps, -10.0f, time, _algorithm);
    }

    std::uint32_t addTracking(
        math::Length firstRange = 1000_m,
        math::Length secondRange = 1000_m,
        std::shared_ptr<const ITrackingAlgorithm> algorithm = nullptr)
    {
        if (!algorithm)
        {
            algorithm = _algorithm;
        }
        const auto id = _repository.add(Target(position(firstRange), 0_mps,
            -10.0f, 0ms, std::move(algorithm)));
        EXPECT_TRUE(_repository.find(id)->confirm(observation(secondRange, 100ms)));
        return id;
    }

    const std::shared_ptr<const ITrackingAlgorithm> _algorithm = std::make_shared<ConstantVelocityAlgorithm>();
    TargetRepository _repository;
    DetectionMatcher _matcher{
        TargetConfig(1000_mps, 30_mps, 7_m, 11.345, 100ms, 1000ms, 3, 10_mps2, 100, 0_mps2)};
    TrackSelector _selector{_repository, _matcher};
};

TEST_F(TrackingAssociationTest, ReturnsSquaredCartesianDistanceScore)
{
    const auto id = addTracking();
    const auto score = _matcher.matchesTracking(observation(1010_m, 1100ms), *_repository.find(id));
    ASSERT_TRUE(score);
    EXPECT_NEAR(*score, 100.0 / 98.0, 1e-12);
}

TEST_F(TrackingAssociationTest, EqualSlantRangesStillUseAzimuthAndElevationDisplacement)
{
    const auto id = addTracking();
    const auto& tracking = *_repository.find(id);
    for (bool elevationOnly : {false, true})
    {
        SCOPED_TRACE(elevationOnly);
        const Target near(math::SphericalPosition(1000_m,
                math::SphericalDirection(elevationOnly ? 0_deg : 1_deg,
                    elevationOnly ? 1_deg : 0_deg)),
            0_mps, -10.0f, 1100ms, _algorithm);
        const Target far(math::SphericalPosition(1000_m,
                math::SphericalDirection(elevationOnly ? 0_deg : 3_deg,
                    elevationOnly ? 3_deg : 0_deg)),
            0_mps, -10.0f, 1100ms, _algorithm);
        // At 1000 m, one degree gives about 17.45 m displacement; three give 52.35 m.
        EXPECT_TRUE(_matcher.matchesTracking(near, tracking));
        EXPECT_FALSE(_matcher.matchesTracking(far, tracking));
    }
}

TEST_F(TrackingAssociationTest, UsesReceiveTimePredictionWithoutTentativeMotionAllowance)
{
    const auto id = addTracking(1000_m, 1010_m);
    const auto& tracking = *_repository.find(id);
    // Two-point initialization gives 100 m/s: 1010 m at 100 ms predicts 1110 m at 1100 ms.
    const auto score = _matcher.matchesTracking(observation(1120_m, 1100ms), tracking);
    ASSERT_TRUE(score);
    EXPECT_NEAR(*score, 100.0 / 98.0, 1e-12);
    EXPECT_FALSE(_matcher.matchesTracking(observation(1000_m, 1100ms), tracking));
    EXPECT_FALSE(_matcher.matchesTracking(observation(1150_m, 1100ms), tracking));
    EXPECT_EQ(tracking.slantRange(), 1010_m);
    EXPECT_EQ(tracking.lastObservationTime(), 100ms);
    EXPECT_EQ(tracking.lastResultTime(), 100ms);
    EXPECT_EQ(tracking.consecutiveMissCount(), 0u);
}

TEST_F(TrackingAssociationTest, SharesConfiguredNdsGateAndAcceptsTheBoundary)
{
    const auto id = addTracking();
    const auto& tracking = *_repository.find(id);
    EXPECT_TRUE(_matcher.matchesTracking(observation(1032_m, 1100ms), tracking));
    EXPECT_FALSE(_matcher.matchesTracking(observation(1034_m, 1100ms), tracking));

    const auto incoming = observation(1010_m, 1100ms);
    const auto score = _matcher.matchesTracking(incoming, tracking);
    ASSERT_TRUE(score);
    const TargetConfig boundaryConfig(
        1000_mps, 30_mps, 7_m, *score, 100ms, 1000ms, 3, 10_mps2, 100, 0_mps2);
    const DetectionMatcher boundaryMatcher(boundaryConfig);
    EXPECT_TRUE(boundaryMatcher.matchesTracking(incoming, tracking));
    EXPECT_TRUE(boundaryMatcher.matchesPosition(incoming, tracking));

    const TargetConfig smallerConfig(1000_mps, 30_mps, 7_m,
        std::nextafter(*score, 0.0), 100ms, 1000ms, 3, 10_mps2, 100, 0_mps2);
    const DetectionMatcher smallerMatcher(smallerConfig);
    EXPECT_FALSE(smallerMatcher.matchesTracking(incoming, tracking));
    EXPECT_FALSE(smallerMatcher.matchesPosition(incoming, tracking));
}

TEST_F(TrackingAssociationTest, UsesInjectedMeasurementNoiseForScoreAndGate)
{
    const auto id = addTracking();
    const auto& tracking = *_repository.find(id);
    const TargetConfig config(
        1000_mps, 30_mps, 2_m, 11.345, 100ms, 1000ms, 3, 10_mps2, 100, 0_mps2);
    const DetectionMatcher matcher(config);
    const auto score = matcher.matchesTracking(observation(1004_m, 1100ms), tracking);
    ASSERT_TRUE(score);
    EXPECT_NEAR(*score, 2.0, 1e-12);
    EXPECT_FALSE(matcher.matchesTracking(observation(1010_m, 1100ms), tracking));
    EXPECT_TRUE(_matcher.matchesTracking(observation(1010_m, 1100ms), tracking));
}

TEST_F(TrackingAssociationTest, UsesPredictedAnisotropicCovariancePlusOneMeasurementNoise)
{
    const auto algorithm = std::make_shared<CovarianceAlgorithm>(
        math::SquareMatrix{{151.0, 0.0, 0.0}, {0.0, 351.0, 0.0}, {0.0, 0.0, 751.0}});
    const auto id = addTracking(1000_m, 1000_m, algorithm);
    const auto& tracking = *_repository.find(id);
    const Target incoming(math::CartesianPosition(10_m, 1020_m, 30_m).toSpherical(),
        0_mps, -10.0f, 1100ms, _algorithm);
    const auto score = _matcher.matchesTracking(incoming, tracking);
    const auto searchScore = _matcher.matchesPosition(incoming, tracking);
    ASSERT_TRUE(score);
    ASSERT_TRUE(searchScore);
    // S = diag(151,351,751) + 49I; r = [10,20,30].
    EXPECT_NEAR(*score, 100.0 / 200.0 + 400.0 / 400.0 + 900.0 / 800.0, 1e-12);
    EXPECT_NEAR(*searchScore, *score, 1e-12);
}

TEST_F(TrackingAssociationTest, RetainsPredictedCrossAxisCorrelations)
{
    const auto algorithm = std::make_shared<CovarianceAlgorithm>(
        math::SquareMatrix{{51.0, 50.0, 0.0}, {50.0, 151.0, 0.0}, {0.0, 0.0, 251.0}});
    const auto id = addTracking(1000_m, 1000_m, algorithm);
    const auto& tracking = *_repository.find(id);
    const Target incoming(math::CartesianPosition(10_m, 1010_m, 0_m).toSpherical(),
        0_mps, -10.0f, 1100ms, _algorithm);
    const auto score = _matcher.matchesTracking(incoming, tracking);
    ASSERT_TRUE(score);
    // The inverse of [[100,50],[50,200]] has determinant 17500.
    EXPECT_NEAR(*score, 8.0 / 7.0, 1e-12);
}

TEST_F(TrackingAssociationTest, PredictionUncertaintyChangesTheGateAtTheRequestedTime)
{
    const auto algorithm = std::make_shared<CovarianceAlgorithm>(
        math::SquareMatrix::identity(3), 100.0);
    const auto id = addTracking(1000_m, 1000_m, algorithm);
    const auto& tracking = *_repository.find(id);
    EXPECT_FALSE(_matcher.matchesTracking(observation(1050_m, 1100ms), tracking));
    const auto later = _matcher.matchesTracking(observation(1050_m, 2100ms), tracking);
    ASSERT_TRUE(later);
    EXPECT_NEAR(*later, 2500.0 / 250.0, 1e-12);
    EXPECT_TRUE(_matcher.matchesPosition(observation(1050_m, 2100ms), tracking));
    EXPECT_EQ(tracking.lastObservationTime(), 100ms);
}

TEST_F(TrackingAssociationTest, PastSearchUsesStoredPositionCovarianceAndMotionAllowance)
{
    const auto algorithm = std::make_shared<CovarianceAlgorithm>(
        math::SquareMatrix{{51.0, 0.0, 0.0}, {0.0, 351.0, 0.0}, {0.0, 0.0, 51.0}});
    const auto id = addTracking(1000_m, 1000_m, algorithm);
    const auto& tracking = *_repository.find(id);
    // 50 ms allows 50 m motion, leaving a 20 m residual along y.
    const auto score = _matcher.matchesPosition(observation(1070_m, 50ms), tracking);
    ASSERT_TRUE(score);
    EXPECT_NEAR(*score, 400.0 / 400.0, 1e-12);
    EXPECT_EQ(tracking.lastObservationTime(), 100ms);
}

TEST(TrackingPredictionCovarianceTest, ValidatesCovarianceOnlyAtConstruction)
{
    EXPECT_NO_THROW(TargetPrediction(0ms, position(1000_m)));
    EXPECT_NO_THROW(TargetPrediction(0ms, position(1000_m), math::SquareMatrix(3)));
    for (std::size_t dimension : {2u, 4u, 9u})
    {
        EXPECT_THROW(TargetPrediction(0ms, position(1000_m),
            math::SquareMatrix::identity(dimension)), std::invalid_argument);
    }
    auto covariance = math::SquareMatrix::identity(3);
    covariance(0, 1) = 0.5;
    EXPECT_THROW(TargetPrediction(0ms, position(1000_m), covariance), std::invalid_argument);
    covariance(1, 0) = 0.5;
    EXPECT_NO_THROW(TargetPrediction(0ms, position(1000_m), covariance));
    covariance(0, 0) = -1.0;
    EXPECT_THROW(TargetPrediction(0ms, position(1000_m), covariance), std::invalid_argument);
    for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::infinity()})
    {
        covariance(0, 0) = invalid;
        EXPECT_THROW(TargetPrediction(0ms, position(1000_m), covariance), std::invalid_argument);
    }
}

TEST_F(TrackingAssociationTest, RejectsWrongStateAndStaleOrDuplicateResults)
{
    const auto unregistered = observation(1000_m, 0ms);
    EXPECT_FALSE(_matcher.matchesTracking(observation(1000_m, 1100ms), unregistered));
    const auto tentativeId = _repository.add(observation(1000_m, 0ms));
    EXPECT_FALSE(_matcher.matchesTracking(observation(1000_m, 1100ms), *_repository.find(tentativeId)));

    const auto id = addTracking();
    auto& tracking = *_repository.find(id);
    EXPECT_FALSE(_matcher.matchesTracking(observation(1000_m, 99ms), tracking));
    EXPECT_FALSE(_matcher.matchesTracking(observation(1000_m, 100ms), tracking));
    ASSERT_TRUE(tracking.recordTrackingMiss(1100ms));
    EXPECT_FALSE(_matcher.matchesTracking(observation(1000_m, 1000ms), tracking));
    EXPECT_FALSE(_matcher.matchesTracking(observation(1000_m, 1100ms), tracking));
    EXPECT_FALSE(_selector.selectObservation(id, 1100ms, {observation(1000_m, 1100ms)}));
    EXPECT_EQ(tracking.consecutiveMissCount(), 1u);
}

TEST_F(TrackingAssociationTest, ReturnsNoMatchWhenPredictionIsUnavailable)
{
    const auto id = addTracking(1000_m, 1000_m, std::make_shared<NoPredictionAlgorithm>());
    const auto incoming = observation(1000_m, 1100ms);
    EXPECT_FALSE(_matcher.matchesTracking(incoming, *_repository.find(id)));
    EXPECT_FALSE(_selector.selectObservation(id, 1100ms, {incoming}));
    EXPECT_EQ(_repository.find(id)->lastResultTime(), 100ms);
}

TEST_F(TrackingAssociationTest, SelectsMinimumScoreIncludingZeroAndKeepsFirstTie)
{
    const auto id = addTracking();
    const auto nearest = _selector.selectObservation(id, 1100ms,
        {observation(1020_m, 1100ms), observation(1005_m, 1100ms), observation(1010_m, 1100ms)});
    ASSERT_TRUE(nearest);
    EXPECT_EQ(*nearest, 1u);
    const auto exact = _selector.selectObservation(id, 1100ms,
        {observation(1010_m, 1100ms), observation(1000_m, 1100ms), observation(1000_m, 1100ms)});
    ASSERT_TRUE(exact);
    EXPECT_EQ(*exact, 1u);
    const auto tied = _selector.selectObservation(id, 1100ms,
        {observation(1010_m, 1100ms), observation(990_m, 1100ms)});
    ASSERT_TRUE(tied);
    EXPECT_EQ(*tied, 0u);
    EXPECT_EQ(_repository.find(id)->lastObservationTime(), 100ms);
}

TEST_F(TrackingAssociationTest, ComparesOnlyTheRequestedIdAndMatchingResultTime)
{
    const auto firstId = addTracking();
    const auto secondId = addTracking(2000_m, 2000_m);
    const std::vector<Target> observations{observation(2000_m, 1100ms), observation(1005_m, 1100ms)};
    const auto first = _selector.selectObservation(firstId, 1100ms, observations);
    const auto second = _selector.selectObservation(secondId, 1100ms, observations);
    ASSERT_TRUE(first);
    ASSERT_TRUE(second);
    EXPECT_EQ(*first, 1u);
    EXPECT_EQ(*second, 0u);
    EXPECT_FALSE(_selector.selectObservation(999, 1100ms, observations));
    EXPECT_FALSE(_selector.selectObservation(firstId, 1200ms, observations));
    EXPECT_EQ(_repository.targets().size(), 2u);
}

TEST(TrackingAssociationControllerTest, OutOfGatePlotsKeepRetryingUntilThreeMissesDeleteTheTarget)
{
    TargetController controller(TargetConfig(1000_mps, 30_mps, 7_m, 11.345, 100ms, 1000ms, 3, 10_mps2, 100, 0_mps2));
    const auto confirmation = controller.handleDetection(result(BeamType::SEARCH, 1, 0ms, {1000_m}));
    ASSERT_TRUE(confirmation);
    const auto tracking = controller.handleDetection(result(BeamType::CONFIRMATION,
        confirmation->requestId, 100ms, {1000_m}));
    ASSERT_TRUE(tracking);

    for (std::uint32_t miss = 1; miss <= 3; ++miss)
    {
        const auto time = 100ms + std::chrono::milliseconds(1000 * miss);
        const auto retry = controller.handleDetection(result(BeamType::TRACKING,
            confirmation->requestId, time, {5000_m}));
        if (miss < 3)
        {
            ASSERT_TRUE(retry);
            EXPECT_EQ(retry->requestId, confirmation->requestId);
            EXPECT_EQ(retry->transmitTime, time + 1000ms);
            const auto events = controller.nextTrackEvents();
            ASSERT_EQ(events.size(), 1u);
            EXPECT_EQ(events[0].timestamp, 100ms);
        }
        else
        {
            EXPECT_FALSE(retry);
            EXPECT_TRUE(controller.nextTrackEvents().empty());
        }
    }
    const auto next = controller.handleDetection(result(BeamType::SEARCH, 1, 3200ms, {1200_m}));
    ASSERT_TRUE(next);
    EXPECT_EQ(next->requestId, 2u);
}

TEST(TrackingAssociationControllerTest, NearestSuccessResetsMissesAndPreservesOtherIdsWithoutRegisteringExtraPlots)
{
    TargetController controller(TargetConfig(1000_mps, 30_mps, 7_m, 11.345, 100ms, 1000ms, 3, 10_mps2, 100, 0_mps2));
    const auto firstConfirmation = controller.handleDetection(result(BeamType::SEARCH, 1, 0ms, {1000_m}));
    ASSERT_TRUE(firstConfirmation);
    ASSERT_TRUE(controller.handleDetection(result(BeamType::CONFIRMATION,
        firstConfirmation->requestId, 100ms, {1000_m})));
    const auto secondConfirmation = controller.handleDetection(result(BeamType::SEARCH, 2, 200ms, {2000_m}));
    ASSERT_TRUE(secondConfirmation);
    ASSERT_TRUE(controller.handleDetection(result(BeamType::CONFIRMATION,
        secondConfirmation->requestId, 300ms, {2000_m})));

    ASSERT_TRUE(controller.handleDetection(result(BeamType::TRACKING,
        firstConfirmation->requestId, 1100ms, {5000_m})));
    const auto recovered = controller.handleDetection(result(BeamType::TRACKING,
        firstConfirmation->requestId, 2100ms, {1010_m, 1000_m, 2000_m, 3000_m}));
    ASSERT_TRUE(recovered);
    EXPECT_EQ(recovered->requestId, firstConfirmation->requestId);
    const auto events = controller.nextTrackEvents();
    ASSERT_EQ(events.size(), 2u);
    EXPECT_EQ(events[0].id, firstConfirmation->requestId);
    EXPECT_EQ(events[0].slantRange, 1000_m);
    EXPECT_EQ(events[0].timestamp, 2100ms);
    EXPECT_EQ(events[1].id, secondConfirmation->requestId);
    EXPECT_EQ(events[1].slantRange, 2000_m);
    EXPECT_EQ(events[1].timestamp, 300ms);

    ASSERT_TRUE(controller.handleDetection(result(BeamType::TRACKING,
        firstConfirmation->requestId, 3100ms, {})));
    ASSERT_TRUE(controller.handleDetection(result(BeamType::TRACKING,
        firstConfirmation->requestId, 4100ms, {})));
    EXPECT_FALSE(controller.handleDetection(result(BeamType::TRACKING,
        firstConfirmation->requestId, 5100ms, {})));
    const auto remaining = controller.nextTrackEvents();
    ASSERT_EQ(remaining.size(), 1u);
    EXPECT_EQ(remaining[0].id, secondConfirmation->requestId);
    EXPECT_EQ(remaining[0].timestamp, 300ms);

    // The other target has not been observed for 4.9 s, so its predicted gate is wider.
    const auto next = controller.handleDetection(result(BeamType::SEARCH, 1, 5200ms, {10000_m}));
    ASSERT_TRUE(next);
    EXPECT_EQ(next->requestId, 3u);
}

} // namespace
} // namespace target
