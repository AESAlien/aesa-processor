#include "algorithm/nds_algorithm.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <limits>

namespace target
{
namespace
{

TEST(NdsAlgorithmTest, CalculatesSquaredDistanceWithCrossAxisCorrelations)
{
    const math::Vector residual{10.0, 10.0, 30.0};
    const math::SquareMatrix covariance{
        {100.0, 50.0, 0.0}, {50.0, 200.0, 0.0}, {0.0, 0.0, 300.0}};
    const auto score = NdsAlgorithm::calculate(residual, covariance);
    ASSERT_TRUE(score);
    // The correlated x/y block contributes 8/7; z contributes 30^2/300.
    EXPECT_NEAR(*score, 29.0 / 7.0, 1e-12);
    const auto algorithm = NdsAlgorithm::from(covariance);
    ASSERT_TRUE(algorithm);
    const auto preparedScore = algorithm->calculate(residual);
    ASSERT_TRUE(preparedScore);
    EXPECT_NEAR(*preparedScore, 29.0 / 7.0, 1e-12);
}

TEST(NdsAlgorithmTest, ReturnsScoresBeyondTheAssociationGate)
{
    const auto score = NdsAlgorithm::calculate(
        math::Vector{1000.0, 0.0, 0.0}, math::SquareMatrix::identity(3) * 98.0);
    ASSERT_TRUE(score);
    EXPECT_NEAR(*score, 1000000.0 / 98.0, 1e-9);
    EXPECT_GT(*score, 11.345);
}

TEST(NdsAlgorithmTest, OwnsAnIndependentFactorizationForRepeatedResiduals)
{
    auto covariance = math::SquareMatrix::identity(3) * 4.0;
    const auto algorithm = NdsAlgorithm::from(covariance);
    ASSERT_TRUE(algorithm);
    covariance(0, 0) = 100.0;

    const auto first = algorithm->calculate(math::Vector{10.0, 0.0, 0.0});
    const auto second = algorithm->calculate(math::Vector{0.0, 20.0, 0.0});
    ASSERT_TRUE(first);
    ASSERT_TRUE(second);
    EXPECT_DOUBLE_EQ(*first, 25.0);
    EXPECT_DOUBLE_EQ(*second, 100.0);
    const auto changed = NdsAlgorithm::calculate(math::Vector{10.0, 0.0, 0.0}, covariance);
    ASSERT_TRUE(changed);
    EXPECT_DOUBLE_EQ(*changed, 1.0);
    EXPECT_FALSE(algorithm->calculate(math::Vector(2)));
    const auto zero = algorithm->calculate(math::Vector(3));
    ASSERT_TRUE(zero);
    EXPECT_DOUBLE_EQ(*zero, 0.0);
}

TEST(NdsAlgorithmTest, RejectsDimensionsOtherThanThree)
{
    const auto covariance = math::SquareMatrix::identity(3);
    const auto algorithm = NdsAlgorithm::from(covariance);
    ASSERT_TRUE(algorithm);
    for (std::size_t dimension : {0u, 2u, 4u})
    {
        SCOPED_TRACE(dimension);
        EXPECT_FALSE(NdsAlgorithm::from(math::SquareMatrix::identity(dimension)));
        EXPECT_FALSE(NdsAlgorithm::calculate(math::Vector(3), math::SquareMatrix::identity(dimension)));
        EXPECT_FALSE(NdsAlgorithm::calculate(math::Vector(dimension), covariance));
        EXPECT_FALSE(algorithm->calculate(math::Vector(dimension)));
    }
}

TEST(NdsAlgorithmTest, RejectsNonfiniteResidualsAndCovariance)
{
    const auto algorithm = NdsAlgorithm::from(math::SquareMatrix::identity(3));
    ASSERT_TRUE(algorithm);
    for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()})
    {
        const math::Vector residual{invalid, 0.0, 0.0};
        EXPECT_FALSE(algorithm->calculate(residual));
        EXPECT_FALSE(NdsAlgorithm::calculate(residual, math::SquareMatrix::identity(3)));
        auto covariance = math::SquareMatrix::identity(3);
        covariance(1, 1) = invalid;
        EXPECT_FALSE(NdsAlgorithm::from(covariance));
        EXPECT_FALSE(NdsAlgorithm::calculate(math::Vector(3), covariance));
    }
}

TEST(NdsAlgorithmTest, RejectsSingularNegativeAndIndefiniteCovariance)
{
    const math::SquareMatrix invalidCovariances[] =
    {
        math::SquareMatrix(3),
        {{-1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}},
        {{1.0, 2.0, 0.0}, {2.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}
    };
    for (const auto& covariance : invalidCovariances)
    {
        EXPECT_FALSE(NdsAlgorithm::from(covariance));
        EXPECT_FALSE(NdsAlgorithm::calculate(math::Vector(3), covariance));
    }
}

TEST(NdsAlgorithmTest, RepairsOnlyAcceptedSymmetryRoundoffWithoutChangingTheInput)
{
    math::SquareMatrix covariance{
        {2.0, 1.0 + 5e-10, 0.0}, {1.0, 2.0, 0.0}, {0.0, 0.0, 3.0}};
    const auto original = covariance;
    const auto score = NdsAlgorithm::calculate(math::Vector{1.0, 1.0, 0.0}, covariance);
    ASSERT_TRUE(score);
    const double symmetricEntry = 1.0 + 0.5 * (covariance(0, 1) - 1.0);
    EXPECT_NEAR(*score, 2.0 / (2.0 + symmetricEntry), 1e-14);
    EXPECT_EQ(covariance(0, 1), original(0, 1));
    EXPECT_EQ(covariance(1, 0), original(1, 0));

    covariance(0, 1) = 1.0 + 2e-9;
    EXPECT_FALSE(NdsAlgorithm::from(covariance));
}

TEST(NdsAlgorithmTest, PreservesTinyScoresAndRejectsUnrepresentableCalculations)
{
    const auto algorithm = NdsAlgorithm::from(math::SquareMatrix::identity(3));
    ASSERT_TRUE(algorithm);
    const double tiny = 1e-160;
    const auto score = algorithm->calculate(math::Vector{tiny, 0.0, 0.0});
    ASSERT_TRUE(score);
    EXPECT_GT(*score, 0.0);
    EXPECT_DOUBLE_EQ(*score, tiny * tiny);

    const math::Vector overflowing{std::numeric_limits<double>::max(), 0.0, 0.0};
    EXPECT_FALSE(algorithm->calculate(overflowing));
    const auto tinyCovariance = NdsAlgorithm::from(math::SquareMatrix::identity(3) * 1e-300);
    ASSERT_TRUE(tinyCovariance);
    EXPECT_FALSE(tinyCovariance->calculate(overflowing));
}

} // namespace
} // namespace target
