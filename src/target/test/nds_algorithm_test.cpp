#include "domain/algorithm/nds_algorithm.hpp"
#include "domain/algorithm/nds_calculation.hpp"
#include "domain/track_state.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <limits>

namespace target
{
namespace
{

using namespace math::literals;
using namespace std::chrono_literals;

TEST(NdsAlgorithmTest, AppliesTravelAllowanceBeforeScoringPositions)
{
    NdsAlgorithm algorithm{MeasurementConfig(2_m)};
    ASSERT_TRUE(algorithm.prepareMeasurements());
    const math::SphericalPosition reference(1000_m, 0_deg, 0_deg);
    const math::SphericalPosition observation(1010_m, 0_deg, 0_deg);
    const auto raw = algorithm.calculate(observation, reference);
    const auto compensated = algorithm.calculate(observation, reference, 6_m);
    const auto covered = algorithm.calculate(observation, reference, 12_m);
    const auto coincident = algorithm.calculate(reference, reference, 0_m);
    ASSERT_TRUE(raw);
    ASSERT_TRUE(compensated);
    ASSERT_TRUE(covered);
    ASSERT_TRUE(coincident);
    EXPECT_DOUBLE_EQ(*raw, 12.5);
    EXPECT_DOUBLE_EQ(*compensated, 2.0);
    EXPECT_DOUBLE_EQ(*covered, 0.0);
    EXPECT_DOUBLE_EQ(*coincident, 0.0);
    EXPECT_FALSE(algorithm.calculate(observation, reference, -1_m));
}

TEST(NdsAlgorithmTest, RepreparesPredictionNoiseAndFallsBackOnlyWhenCovarianceIsAbsent)
{
    NdsAlgorithm algorithm{MeasurementConfig(2_m)};
    const math::SphericalPosition position(1000_m, 0_deg, 0_deg);
    const math::Vector residual({4.0, 0.0, 0.0});
    EXPECT_FALSE(algorithm.calculate(residual));
    ASSERT_TRUE(algorithm.preparePrediction(TargetPrediction(1ms, position)));
    const auto fallback = algorithm.calculate(residual);
    ASSERT_TRUE(fallback);
    EXPECT_DOUBLE_EQ(*fallback, 2.0);
    ASSERT_TRUE(algorithm.preparePrediction(TargetPrediction(1ms, position, math::SquareMatrix::identity(3) * 12.0)));
    const auto known = algorithm.calculate(residual);
    ASSERT_TRUE(known);
    EXPECT_DOUBLE_EQ(*known, 1.0);
    ASSERT_TRUE(algorithm.preparePrediction(TargetPrediction(1ms, position, math::SquareMatrix(3))));
    const auto zero = algorithm.calculate(residual);
    ASSERT_TRUE(zero);
    EXPECT_DOUBLE_EQ(*zero, 4.0);
    ASSERT_TRUE(algorithm.prepareMeasurements());
    const auto measurements = algorithm.calculate(residual);
    ASSERT_TRUE(measurements);
    EXPECT_DOUBLE_EQ(*measurements, 2.0);
}

TEST(NdsAlgorithmTest, FailedPreparationClearsEarlierStateAndAllowsRecovery)
{
    NdsAlgorithm algorithm{MeasurementConfig(2_m)};
    const math::SphericalPosition position(1000_m, 0_deg, 0_deg);
    const math::Vector residual({4.0, 0.0, 0.0});
    ASSERT_TRUE(algorithm.prepareMeasurements());
    ASSERT_TRUE(algorithm.calculate(residual));
    const math::SquareMatrix indefinite({{1.0, 100.0, 0.0}, {100.0, 1.0, 0.0}, {0.0, 0.0, 1.0}});
    EXPECT_FALSE(algorithm.preparePrediction(TargetPrediction(1ms, position, indefinite)));
    EXPECT_FALSE(algorithm.calculate(residual));
    EXPECT_FALSE(algorithm.calculate(position, position));
    ASSERT_TRUE(algorithm.prepareMeasurements());
    const auto recovered = algorithm.calculate(residual);
    ASSERT_TRUE(recovered);
    EXPECT_DOUBLE_EQ(*recovered, 2.0);
}

TEST(NdsCalculationTest, CalculatesSquaredDistanceWithCrossAxisCorrelations)
{
    const math::Vector residual({10.0, 10.0, 30.0});
    const math::SquareMatrix covariance({
        {100.0, 50.0, 0.0},
        {50.0, 200.0, 0.0},
        {0.0, 0.0, 300.0}
    });
    const auto score = NdsCalculation::calculate(residual, covariance);
    ASSERT_TRUE(score);
    // The correlated x/y block contributes 8/7; z contributes 30^2/300.
    EXPECT_NEAR(*score, 29.0 / 7.0, 1e-12);
    const auto algorithm = NdsCalculation::from(covariance);
    ASSERT_TRUE(algorithm);
    const NdsCalculation& scorer = *algorithm;
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

    const NdsCalculation& scorer = *algorithm;
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
    for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::infinity(),
             -std::numeric_limits<double>::infinity()})
    {
        const math::Vector residual({invalid, 0.0, 0.0});
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
    math::SquareMatrix covariance({
        {2.0, 1.0 + 5e-10, 0.0},
        {1.0, 2.0, 0.0},
        {0.0, 0.0, 3.0}
    });
    const auto original = covariance;
    const auto score = NdsCalculation::calculate(math::Vector({1.0, 1.0, 0.0}), covariance);
    ASSERT_TRUE(score);
    const double symmetricEntry = 1.0 + 0.5 * (covariance(0, 1) - 1.0);
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
    const double tiny = 1e-160;
    const auto score = algorithm->calculate(math::Vector({tiny, 0.0, 0.0}));
    ASSERT_TRUE(score);
    EXPECT_GT(*score, 0.0);
    EXPECT_DOUBLE_EQ(*score, tiny * tiny);

    const math::Vector overflowing({std::numeric_limits<double>::max(), 0.0, 0.0});
    EXPECT_FALSE(algorithm->calculate(overflowing));
    const auto tinyCovariance = NdsCalculation::from(math::SquareMatrix::identity(3) * 1e-300);
    ASSERT_TRUE(tinyCovariance);
    EXPECT_FALSE(tinyCovariance->calculate(overflowing));
}

} // namespace
} // namespace target
