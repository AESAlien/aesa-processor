#include "math/quantity/length.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <type_traits>

static_assert(!std::is_default_constructible_v<math::Length>);
static_assert(!std::is_constructible_v<math::Length, double>);

TEST(LengthTest, StoresSignedMeters)
{
    const double values[] = {-1234.567, -0.0, 0.0, 0.001, 1234.567};

    for (const double value : values)
    {
        const auto length = math::Length::fromMeters(value);
        EXPECT_DOUBLE_EQ(length.m(), value);
        EXPECT_DOUBLE_EQ(length.km(), value / 1000.0);
        EXPECT_DOUBLE_EQ((-length).m(), -value);
    }
}

TEST(LengthTest, ConvertsSignedKilometersToMeters)
{
    const auto negative = math::Length::fromKilometers(-1.25);
    const auto positive = math::Length::fromKilometers(2.5);

    EXPECT_DOUBLE_EQ(negative.m(), -1250.0);
    EXPECT_DOUBLE_EQ(positive.m(), 2500.0);
}

TEST(LengthTest, AddsAndSubtractsLengths)
{
    const auto left = math::Length::fromMeters(3.5);
    const auto right = math::Length::fromMeters(1.25);

    static_assert(std::is_same_v<decltype(left + right), math::Length>);
    static_assert(std::is_same_v<decltype(left - right), math::Length>);
    EXPECT_DOUBLE_EQ((left + right).m(), 4.75);
    EXPECT_DOUBLE_EQ((left - right).m(), 2.25);
}

TEST(LengthTest, RejectsNonFiniteAdditionAndSubtractionResults)
{
    const auto maximum = math::Length::fromMeters(std::numeric_limits<double>::max());

    EXPECT_THROW(maximum + maximum, std::invalid_argument);
    EXPECT_THROW(maximum - (-maximum), std::invalid_argument);
}

TEST(LengthTest, MultipliesAndDividesByScalars)
{
    const auto length = math::Length::fromMeters(3.0);

    static_assert(std::is_same_v<decltype(length * 2.0), math::Length>);
    static_assert(std::is_same_v<decltype(2.0 * length), math::Length>);
    static_assert(std::is_same_v<decltype(length / 2.0), math::Length>);
    EXPECT_DOUBLE_EQ((length * -2.0).m(), -6.0);
    EXPECT_DOUBLE_EQ((-2.0 * length).m(), -6.0);
    EXPECT_DOUBLE_EQ((length / 2.0).m(), 1.5);
}

TEST(LengthTest, RejectsInvalidScalarOperations)
{
    const auto maximum = math::Length::fromMeters(std::numeric_limits<double>::max());

    EXPECT_THROW(maximum * 2.0, std::invalid_argument);
    EXPECT_THROW(maximum / 0.0, std::domain_error);
    EXPECT_THROW(maximum / -0.0, std::domain_error);
}

TEST(LengthTest, AppliesCompoundArithmetic)
{
    auto length = math::Length::fromMeters(8.0);

    EXPECT_EQ(&(length += math::Length::fromMeters(2.0)), &length);
    EXPECT_DOUBLE_EQ(length.m(), 10.0);
    EXPECT_EQ(&(length -= math::Length::fromMeters(4.0)), &length);
    EXPECT_DOUBLE_EQ(length.m(), 6.0);
    EXPECT_EQ(&(length *= -2.0), &length);
    EXPECT_DOUBLE_EQ(length.m(), -12.0);
    EXPECT_EQ(&(length /= 3.0), &length);
    EXPECT_DOUBLE_EQ(length.m(), -4.0);
}

TEST(LengthTest, CompoundArithmeticPreservesValueOnFailure)
{
    const auto maximum = math::Length::fromMeters(std::numeric_limits<double>::max());
    auto length = maximum;

    EXPECT_THROW(length += maximum, std::invalid_argument);
    EXPECT_EQ(length, maximum);
    EXPECT_THROW(length -= -maximum, std::invalid_argument);
    EXPECT_EQ(length, maximum);
    EXPECT_THROW(length *= 2.0, std::invalid_argument);
    EXPECT_EQ(length, maximum);
    EXPECT_THROW(length /= 0.0, std::domain_error);
    EXPECT_EQ(length, maximum);
}

TEST(LengthTest, CreatesLengthsFromMeterLiterals)
{
    using namespace math::literals;

    static_assert(std::is_same_v<decltype(2_m), math::Length>);
    static_assert(std::is_same_v<decltype(2.5_m), math::Length>);
    EXPECT_DOUBLE_EQ((2_m).m(), 2.0);
    EXPECT_DOUBLE_EQ((-2.5_m).m(), -2.5);
}

TEST(LengthTest, CreatesLengthsFromKilometerLiterals)
{
    using namespace math::literals;

    static_assert(std::is_same_v<decltype(2_km), math::Length>);
    static_assert(std::is_same_v<decltype(2.5_km), math::Length>);
    EXPECT_DOUBLE_EQ((2_km).km(), 2.0);
    EXPECT_DOUBLE_EQ((-2.5_km).km(), -2.5);
}

TEST(LengthTest, ComparesSignedValues)
{
    const auto negative = math::Length::fromMeters(-1.0);
    const auto zero = math::Length::fromMeters(0.0);
    const auto positive = math::Length::fromMeters(1.0);

    EXPECT_LT(negative, zero);
    EXPECT_LT(zero, positive);
    EXPECT_TRUE(negative.equals(
        math::Length::fromMeters(-1.25),
        math::QuantityTolerance{math::Length::fromMeters(0.25), 0.0}));
}

TEST(LengthTest, AbsPreservesLengthTypeAndReturnsNonnegativeValue)
{
    static_assert(std::is_same_v<
        decltype(math::abs(math::Length::fromMeters(-1.0))),
        math::Length>);
    EXPECT_DOUBLE_EQ(math::abs(math::Length::fromKilometers(-2.0)).km(), 2.0);
}

TEST(LengthTest, RejectsNonFiniteValues)
{
    for (double value : {
             std::numeric_limits<double>::infinity(),
             -std::numeric_limits<double>::infinity(),
             std::numeric_limits<double>::quiet_NaN()})
    {
        EXPECT_THROW(math::Length::fromMeters(value), std::invalid_argument);
        EXPECT_THROW(math::Length::fromKilometers(value), std::invalid_argument);
        EXPECT_THROW(math::literals::operator""_m(static_cast<long double>(value)), std::invalid_argument);
        EXPECT_THROW(math::literals::operator""_km(static_cast<long double>(value)), std::invalid_argument);
    }
}

TEST(LengthTest, RejectsOverflowDuringKilometerConversion)
{
    const double maxValue = std::numeric_limits<double>::max();
    EXPECT_THROW(math::Length::fromKilometers(maxValue), std::invalid_argument);
    EXPECT_THROW(math::Length::fromKilometers(-maxValue), std::invalid_argument);
}
