#include "association_selection_helpers.hpp"
#include "domain/association/nds_algorithm.hpp"
#include "domain/association/nds_calculation.hpp"
#include "domain/confirmed_track.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <limits>

namespace target
{
namespace
{

using namespace math::literals;
using namespace std::chrono_literals;

ConfirmedTrack stationaryTrack(std::shared_ptr<const ITrackingAlgorithm> algorithm)
{
    const auto position = math::CartesianPosition(0_m, 1000_m, 0_m);
    const auto state = TrackState(
        position.toSpherical(), 0_mps, -10.0f, 100ms, position, math::CartesianVelocity(0_mps, 0_mps, 0_mps)
    );
    return ConfirmedTrack(1, 0ms, state, std::move(algorithm));
}

TEST(NdsAlgorithmTest, AppliesTravelAllowanceBeforeScoringPositions)
{
    auto algorithm = NdsAlgorithm{MeasurementConfig(2_m)};
    const auto calculation = algorithm.prepareMeasurements();
    ASSERT_TRUE(calculation);
    const auto reference = math::SphericalPosition(1000_m, 0_deg, 0_deg);
    const auto observation = math::SphericalPosition(1010_m, 0_deg, 0_deg);
    const auto raw = calculation->calculate(observation, reference);
    const auto compensated = calculation->calculate(observation, reference, math::Distance(6_m));
    const auto covered = calculation->calculate(observation, reference, math::Distance(12_m));
    const auto coincident = calculation->calculate(reference, reference, math::Distance(0_m));
    ASSERT_TRUE(raw);
    ASSERT_TRUE(compensated);
    ASSERT_TRUE(covered);
    ASSERT_TRUE(coincident);
    EXPECT_DOUBLE_EQ(*raw, 12.5);
    EXPECT_DOUBLE_EQ(*compensated, 2.0);
    EXPECT_DOUBLE_EQ(*covered, 0.0);
    EXPECT_DOUBLE_EQ(*coincident, 0.0);
    EXPECT_THROW(math::Distance(-1_m), std::invalid_argument);
}

TEST(NdsAlgorithmTest, PreparedCalculationsRemainIndependentAcrossPreparations)
{
    auto algorithm = NdsAlgorithm{MeasurementConfig(2_m)};
    const auto residual = math::Vector({4.0, 0.0, 0.0});
    const auto withoutCovariance = stationaryTrack(std::make_shared<ConstantVelocityAlgorithm>());
    const auto fallbackPrediction = algorithm.predictAndPrepare(withoutCovariance, 1100ms);
    ASSERT_TRUE(fallbackPrediction);
    const auto fallback = fallbackPrediction->second->calculate(residual);
    ASSERT_TRUE(fallback);
    EXPECT_DOUBLE_EQ(*fallback, 2.0);
    const auto withCovariance = stationaryTrack(std::make_shared<test_support::CovarianceAlgorithm>(
        math::SquareMatrix::identity(3) * 12.0
    ));
    const auto knownPrediction = algorithm.predictAndPrepare(withCovariance, 1100ms);
    ASSERT_TRUE(knownPrediction);
    const auto known = knownPrediction->second->calculate(residual);
    ASSERT_TRUE(known);
    EXPECT_DOUBLE_EQ(*known, 1.0);
    const auto calculation = algorithm.prepareMeasurements();
    ASSERT_TRUE(calculation);
    const auto measurements = calculation->calculate(residual);
    ASSERT_TRUE(measurements);
    EXPECT_DOUBLE_EQ(*measurements, 2.0);
    EXPECT_EQ(fallbackPrediction->second->calculate(residual), fallback);
    EXPECT_EQ(knownPrediction->second->calculate(residual), known);
}

TEST(NdsAlgorithmTest, PredictionCovarianceIsValidatedBeforePreparation)
{
    const auto indefinite = math::SquareMatrix({{1.0, 100.0, 0.0}, {100.0, 1.0, 0.0}, {0.0, 0.0, 1.0}});
    const auto track = stationaryTrack(std::make_shared<test_support::CovarianceAlgorithm>(indefinite));
    auto algorithm = NdsAlgorithm{MeasurementConfig(2_m)};
    EXPECT_THROW(algorithm.predictAndPrepare(track, 1100ms), std::invalid_argument);
    EXPECT_TRUE(algorithm.prepareMeasurements());
}

TEST(NdsAlgorithmTest, PreparedCalculationSurvivesFailedPreparationAndAlgorithmDestruction)
{
    const auto residual = math::Vector({4.0, 0.0, 0.0});
    AssociationCalculation calculation;
    std::optional<double> originalScore;
    {
        const auto algorithm = NdsAlgorithm{MeasurementConfig(2_m)};
        calculation = algorithm.prepareMeasurements();
        ASSERT_TRUE(calculation);
        originalScore = calculation->calculate(residual);
        ASSERT_TRUE(originalScore);
        EXPECT_DOUBLE_EQ(*originalScore, 2.0);
        const auto track = stationaryTrack(std::make_shared<ConstantVelocityAlgorithm>());
        EXPECT_FALSE(algorithm.predictAndPrepare(track, 0ms));
        const auto invalidTrack = stationaryTrack(std::make_shared<test_support::CovarianceAlgorithm>(
            math::SquareMatrix(3)
        ));
        EXPECT_THROW(algorithm.prepareFromLastObservation(invalidTrack), std::invalid_argument);
        EXPECT_EQ(calculation->calculate(residual), originalScore);
    }
    EXPECT_EQ(calculation->calculate(residual), originalScore);
}

TEST(NdsCalculationTest, CalculatesSquaredDistanceWithCrossAxisCorrelations)
{
    const auto residual = math::Vector({10.0, 10.0, 30.0});
    const auto covariance = math::SquareMatrix({
        {100.0, 50.0, 0.0},
        {50.0, 200.0, 0.0},
        {0.0, 0.0, 300.0}
    });
    const auto score = NdsCalculation::calculate(residual, covariance);
    ASSERT_TRUE(score);
    // 서로 상관된 x축과 y축의 기여는 8/7이고, 독립인 z축의 기여는 30^2/300이다.
    EXPECT_NEAR(*score, 29.0 / 7.0, 1e-12);
    const auto algorithm = NdsCalculation::from(covariance);
    ASSERT_TRUE(algorithm);
    const auto& scorer = *algorithm;
    const auto preparedScore = scorer.calculate(residual);
    ASSERT_TRUE(preparedScore);
    EXPECT_NEAR(*preparedScore, 29.0 / 7.0, 1e-12);
}

TEST(NdsCalculationTest, ReturnsScoresBeyondTheAssociationGate)
{
    const auto score = NdsCalculation::calculate(
        math::Vector({1000.0, 0.0, 0.0}),
        math::SquareMatrix::identity(3) * 98.0
    );
    ASSERT_TRUE(score);
    EXPECT_NEAR(*score, 1000000.0 / 98.0, 1e-9);
    EXPECT_GT(*score, 11.345);
}

TEST(NdsCalculationTest, OwnsAnIndependentFactorizationForRepeatedResiduals)
{
    auto covariance = math::SquareMatrix::identity(3) * 4.0;
    const auto algorithm = NdsCalculation::from(covariance);
    ASSERT_TRUE(algorithm);
    covariance(0, 0) = 100.0;

    const auto& scorer = *algorithm;
    const auto first = scorer.calculate(math::Vector({10.0, 0.0, 0.0}));
    const auto second = scorer.calculate(math::Vector({0.0, 20.0, 0.0}));
    ASSERT_TRUE(first);
    ASSERT_TRUE(second);
    EXPECT_DOUBLE_EQ(*first, 25.0);
    EXPECT_DOUBLE_EQ(*second, 100.0);
    const auto changed = NdsCalculation::calculate(math::Vector({10.0, 0.0, 0.0}), covariance);
    ASSERT_TRUE(changed);
    EXPECT_DOUBLE_EQ(*changed, 1.0);
    EXPECT_FALSE(scorer.calculate(math::Vector(2)));
    const auto zero = scorer.calculate(math::Vector(3));
    ASSERT_TRUE(zero);
    EXPECT_DOUBLE_EQ(*zero, 0.0);
}

TEST(NdsCalculationTest, RejectsDimensionsOtherThanThree)
{
    const auto covariance = math::SquareMatrix::identity(3);
    const auto algorithm = NdsCalculation::from(covariance);
    ASSERT_TRUE(algorithm);
    for (std::size_t dimension : {0u, 2u, 4u})
    {
        SCOPED_TRACE(dimension);
        EXPECT_FALSE(NdsCalculation::from(math::SquareMatrix::identity(dimension)));
        EXPECT_FALSE(NdsCalculation::calculate(math::Vector(3), math::SquareMatrix::identity(dimension)));
        EXPECT_FALSE(NdsCalculation::calculate(math::Vector(dimension), covariance));
        EXPECT_FALSE(algorithm->calculate(math::Vector(dimension)));
    }
}

TEST(NdsCalculationTest, RejectsNonfiniteResidualsAndCovariance)
{
    const auto algorithm = NdsCalculation::from(math::SquareMatrix::identity(3));
    ASSERT_TRUE(algorithm);
    for (auto invalid : {std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::infinity(),
             -std::numeric_limits<double>::infinity()})
    {
        const auto residual = math::Vector({invalid, 0.0, 0.0});
        EXPECT_FALSE(algorithm->calculate(residual));
        EXPECT_FALSE(NdsCalculation::calculate(residual, math::SquareMatrix::identity(3)));
        auto covariance = math::SquareMatrix::identity(3);
        covariance(1, 1) = invalid;
        EXPECT_FALSE(NdsCalculation::from(covariance));
        EXPECT_FALSE(NdsCalculation::calculate(math::Vector(3), covariance));
    }
}

TEST(NdsCalculationTest, RejectsSingularNegativeAndIndefiniteCovariance)
{
    const math::SquareMatrix invalidCovariances[] =
    {
        math::SquareMatrix(3),
        {{-1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}},
        {{1.0, 2.0, 0.0}, {2.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}
    };
    for (const auto& covariance : invalidCovariances)
    {
        EXPECT_FALSE(NdsCalculation::from(covariance));
        EXPECT_FALSE(NdsCalculation::calculate(math::Vector(3), covariance));
    }
}

TEST(NdsCalculationTest, RepairsOnlyAcceptedSymmetryRoundoffWithoutChangingTheInput)
{
    auto covariance = math::SquareMatrix({
        {2.0, 1.0 + 5e-10, 0.0},
        {1.0, 2.0, 0.0},
        {0.0, 0.0, 3.0}
    });
    const auto original = covariance;
    const auto score = NdsCalculation::calculate(math::Vector({1.0, 1.0, 0.0}), covariance);
    ASSERT_TRUE(score);
    const auto symmetricEntry = 1.0 + 0.5 * (covariance(0, 1) - 1.0);
    EXPECT_NEAR(*score, 2.0 / (2.0 + symmetricEntry), 1e-14);
    EXPECT_EQ(covariance(0, 1), original(0, 1));
    EXPECT_EQ(covariance(1, 0), original(1, 0));

    covariance(0, 1) = 1.0 + 2e-9;
    EXPECT_FALSE(NdsCalculation::from(covariance));
}

TEST(NdsCalculationTest, PreservesTinyScoresAndRejectsUnrepresentableCalculations)
{
    const auto algorithm = NdsCalculation::from(math::SquareMatrix::identity(3));
    ASSERT_TRUE(algorithm);
    const auto tiny = 1e-160;
    const auto score = algorithm->calculate(math::Vector({tiny, 0.0, 0.0}));
    ASSERT_TRUE(score);
    EXPECT_GT(*score, 0.0);
    EXPECT_DOUBLE_EQ(*score, tiny * tiny);

    const auto overflowing = math::Vector({std::numeric_limits<double>::max(), 0.0, 0.0});
    EXPECT_FALSE(algorithm->calculate(overflowing));
    const auto tinyCovariance = NdsCalculation::from(math::SquareMatrix::identity(3) * 1e-300);
    ASSERT_TRUE(tinyCovariance);
    EXPECT_FALSE(tinyCovariance->calculate(overflowing));
}

} // namespace
} // namespace target
