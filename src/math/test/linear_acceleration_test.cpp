#include "math/quantity/linear_acceleration.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <type_traits>

using namespace math::literals;
using namespace std::chrono_literals;

static_assert(!std::is_default_constructible_v<math::LinearAcceleration>);
static_assert(!std::is_constructible_v<math::LinearAcceleration, double>);

TEST(LinearAccelerationTest, PreservesSignedMetersPerSecondSquared)
{
    const auto acceleration = math::LinearAcceleration::fromMetersPerSecondSquared(-2.5);

    EXPECT_DOUBLE_EQ(acceleration.mps2(), -2.5);
    EXPECT_EQ(-acceleration, 2.5_mps2);
}

TEST(LinearAccelerationTest, SupportsQuantityArithmetic)
{
    static_assert(std::is_same_v<decltype(3_mps2 + 2_mps2), math::LinearAcceleration>);
    static_assert(std::is_same_v<decltype(3_mps2 * 2.0), math::LinearAcceleration>);

    EXPECT_EQ(3_mps2 + 2_mps2, 5_mps2);
    EXPECT_EQ(3_mps2 - 2_mps2, 1_mps2);
    EXPECT_EQ(2.0 * 3_mps2, 6_mps2);
    EXPECT_EQ(3_mps2 / 2.0, 1.5_mps2);
}

TEST(LinearAccelerationTest, RejectsInvalidScalarArithmetic)
{
    const auto maximum = math::LinearAcceleration::fromMetersPerSecondSquared(
        std::numeric_limits<double>::max());

    EXPECT_THROW(maximum + maximum, std::invalid_argument);
    EXPECT_THROW(maximum - (-maximum), std::invalid_argument);
    EXPECT_THROW(maximum * 2.0, std::invalid_argument);
    EXPECT_THROW(maximum / 0.0, std::domain_error);
    EXPECT_THROW(maximum / -0.0, std::domain_error);
}

TEST(LinearAccelerationTest, AppliesCompoundArithmetic)
{
    auto acceleration = 8_mps2;

    EXPECT_EQ(&(acceleration += 2_mps2), &acceleration);
    EXPECT_EQ(acceleration, 10_mps2);
    EXPECT_EQ(&(acceleration -= 4_mps2), &acceleration);
    EXPECT_EQ(acceleration, 6_mps2);
    EXPECT_EQ(&(acceleration *= -2.0), &acceleration);
    EXPECT_EQ(acceleration, -12_mps2);
    EXPECT_EQ(&(acceleration /= 3.0), &acceleration);
    EXPECT_EQ(acceleration, -4_mps2);
}

TEST(LinearAccelerationTest, CompoundArithmeticPreservesValueOnFailure)
{
    const auto maximum = math::LinearAcceleration::fromMetersPerSecondSquared(
        std::numeric_limits<double>::max());
    auto acceleration = maximum;

    EXPECT_THROW(acceleration += maximum, std::invalid_argument);
    EXPECT_EQ(acceleration, maximum);
    EXPECT_THROW(acceleration -= -maximum, std::invalid_argument);
    EXPECT_EQ(acceleration, maximum);
    EXPECT_THROW(acceleration *= 2.0, std::invalid_argument);
    EXPECT_EQ(acceleration, maximum);
    EXPECT_THROW(acceleration /= 0.0, std::domain_error);
    EXPECT_EQ(acceleration, maximum);
}

TEST(LinearAccelerationTest, CreatesMetersPerSecondSquaredLiterals)
{
    static_assert(std::is_same_v<decltype(2_mps2), math::LinearAcceleration>);
    static_assert(std::is_same_v<decltype(2.5_mps2), math::LinearAcceleration>);
    EXPECT_DOUBLE_EQ((2_mps2).mps2(), 2.0);
    EXPECT_DOUBLE_EQ((-2.5_mps2).mps2(), -2.5);
}

TEST(LinearAccelerationTest, CalculatesVelocityFromAccelerationAndDuration)
{
    static_assert(std::is_same_v<decltype(2_mps2 * 500ms), math::LinearVelocity>);

    EXPECT_EQ(2_mps2 * 500ms, 1_mps);
    EXPECT_EQ(-2_mps2 * 3s, -6_mps);
    EXPECT_EQ(2_mps2 * -3s, -6_mps);
}

TEST(LinearAccelerationTest, RejectsNonfiniteValuesAndResults)
{
    const double maximum = std::numeric_limits<double>::max();

    EXPECT_THROW(
        math::LinearAcceleration::fromMetersPerSecondSquared(
            std::numeric_limits<double>::infinity()),
        std::invalid_argument);
    EXPECT_THROW(
        math::LinearAcceleration::fromMetersPerSecondSquared(maximum) * 2s,
        std::invalid_argument);
    EXPECT_THROW(
        1_mps2 * std::chrono::duration<double>{
            std::numeric_limits<double>::quiet_NaN()},
        std::invalid_argument);
    EXPECT_THROW(
        math::literals::operator""_mps2(
            static_cast<long double>(std::numeric_limits<double>::infinity())),
        std::invalid_argument);
}

TEST(LinearAccelerationTest, AbsReturnsNonnegativeAcceleration)
{
    EXPECT_EQ(math::abs(-2_mps2), 2_mps2);
    EXPECT_EQ(math::abs(2_mps2), 2_mps2);
}
