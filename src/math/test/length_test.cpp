#include "math/quantity/length.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <type_traits>

using namespace aesa_processor::math::literals;

static_assert(!std::is_default_constructible_v<aesa_processor::math::Length>);
static_assert(!std::is_constructible_v<aesa_processor::math::Length, double>);



// 객체 생성과 상태 보존 규칙을 검증한다.
TEST(LengthTest, StoresSignedMeters)
{
    const double values[] = {-1234.567, -0.0, 0.0, 0.001, 1234.567};

    for (const double value : values)
    {
        const auto length = aesa_processor::math::Length::fromMeters(value);
        EXPECT_DOUBLE_EQ(length.m(), value);
        EXPECT_DOUBLE_EQ(length.km(), value / 1000.0);
        EXPECT_DOUBLE_EQ((-length).m(), -value);
    }
}

// 객체 생성과 상태 보존 규칙을 검증한다.
TEST(LengthTest, CreatesLengthsFromMeterLiterals)
{
    using namespace aesa_processor::math::literals;

    static_assert(std::is_same_v<decltype(2_m), aesa_processor::math::Length>);
    static_assert(std::is_same_v<decltype(2.5_m), aesa_processor::math::Length>);
    EXPECT_DOUBLE_EQ((2_m).m(), 2.0);
    EXPECT_DOUBLE_EQ((-2.5_m).m(), -2.5);
}

// 객체 생성과 상태 보존 규칙을 검증한다.
TEST(LengthTest, CreatesLengthsFromKilometerLiterals)
{
    using namespace aesa_processor::math::literals;

    static_assert(std::is_same_v<decltype(2_km), aesa_processor::math::Length>);
    static_assert(std::is_same_v<decltype(2.5_km), aesa_processor::math::Length>);
    EXPECT_DOUBLE_EQ((2_km).km(), 2.0);
    EXPECT_DOUBLE_EQ((-2.5_km).km(), -2.5);
}

// 값의 변환과 정규화 결과를 검증한다.
TEST(LengthTest, ConvertsSignedKilometersToMeters)
{
    const auto negative = -1.25_km;
    const auto positive = 2.5_km;

    EXPECT_DOUBLE_EQ(negative.m(), -1250.0);
    EXPECT_DOUBLE_EQ(positive.m(), 2500.0);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(LengthTest, AddsAndSubtractsLengths)
{
    const auto left = 3.5_m;
    const auto right = 1.25_m;

    static_assert(std::is_same_v<decltype(left + right), aesa_processor::math::Length>);
    static_assert(std::is_same_v<decltype(left - right), aesa_processor::math::Length>);
    EXPECT_DOUBLE_EQ((left + right).m(), 4.75);
    EXPECT_DOUBLE_EQ((left - right).m(), 2.25);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(LengthTest, MultipliesAndDividesByScalars)
{
    const auto length = 3.0_m;

    static_assert(std::is_same_v<decltype(length * 2.0), aesa_processor::math::Length>);
    static_assert(std::is_same_v<decltype(2.0 * length), aesa_processor::math::Length>);
    static_assert(std::is_same_v<decltype(length / 2.0), aesa_processor::math::Length>);
    EXPECT_DOUBLE_EQ((length * -2.0).m(), -6.0);
    EXPECT_DOUBLE_EQ((-2.0 * length).m(), -6.0);
    EXPECT_DOUBLE_EQ((length / 2.0).m(), 1.5);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(LengthTest, AppliesCompoundArithmetic)
{
    auto length = 8.0_m;

    EXPECT_EQ(&(length += 2.0_m), &length);
    EXPECT_DOUBLE_EQ(length.m(), 10.0);
    EXPECT_EQ(&(length -= 4.0_m), &length);
    EXPECT_DOUBLE_EQ(length.m(), 6.0);
    EXPECT_EQ(&(length *= -2.0), &length);
    EXPECT_DOUBLE_EQ(length.m(), -12.0);
    EXPECT_EQ(&(length /= 3.0), &length);
    EXPECT_DOUBLE_EQ(length.m(), -4.0);
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(LengthTest, ComparesSignedValues)
{
    const auto negative = -1.0_m;
    const auto zero = 0.0_m;
    const auto positive = 1.0_m;

    EXPECT_LT(negative, zero);
    EXPECT_LT(zero, positive);
    EXPECT_TRUE(negative.equals(
        -1.25_m,
        aesa_processor::math::QuantityTolerance{0.25_m, 0.0}));
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(LengthTest, AbsPreservesLengthTypeAndReturnsNonnegativeValue)
{
    static_assert(std::is_same_v<
        decltype(aesa_processor::math::abs(-1.0_m)),
        aesa_processor::math::Length>);
    EXPECT_DOUBLE_EQ(aesa_processor::math::abs(-2.0_km).km(), 2.0);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(LengthTest, RejectsNonFiniteAdditionAndSubtractionResults)
{
    const auto maximum = aesa_processor::math::Length::fromMeters(std::numeric_limits<double>::max());

    EXPECT_THROW(maximum + maximum, std::invalid_argument);
    EXPECT_THROW(maximum - (-maximum), std::invalid_argument);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(LengthTest, RejectsInvalidScalarOperations)
{
    const auto maximum = aesa_processor::math::Length::fromMeters(std::numeric_limits<double>::max());

    EXPECT_THROW(maximum * 2.0, std::invalid_argument);
    EXPECT_THROW(maximum / 0.0, std::domain_error);
    EXPECT_THROW(maximum / -0.0, std::domain_error);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(LengthTest, CompoundArithmeticPreservesValueOnFailure)
{
    const auto maximum = aesa_processor::math::Length::fromMeters(std::numeric_limits<double>::max());
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

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(LengthTest, RejectsNonFiniteValues)
{
    for (double value : {
             std::numeric_limits<double>::infinity(),
             -std::numeric_limits<double>::infinity(),
             std::numeric_limits<double>::quiet_NaN()})
    {
        EXPECT_THROW(aesa_processor::math::Length::fromMeters(value), std::invalid_argument);
        EXPECT_THROW(aesa_processor::math::Length::fromKilometers(value), std::invalid_argument);
        EXPECT_THROW(aesa_processor::math::literals::operator""_m(static_cast<long double>(value)), std::invalid_argument);
        EXPECT_THROW(aesa_processor::math::literals::operator""_km(static_cast<long double>(value)), std::invalid_argument);
    }
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(LengthTest, RejectsOverflowDuringKilometerConversion)
{
    const double maxValue = std::numeric_limits<double>::max();
    EXPECT_THROW(aesa_processor::math::Length::fromKilometers(maxValue), std::invalid_argument);
    EXPECT_THROW(aesa_processor::math::Length::fromKilometers(-maxValue), std::invalid_argument);
}
