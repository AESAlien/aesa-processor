#include "math/quantity/angle.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <type_traits>

using namespace math::literals;

// 단위를 명시하는 팩터리를 거치도록 기본 생성과 double 직접 생성을 금지한다.
static_assert(!std::is_default_constructible<math::Angle>::value,
    "Angles must be created through a unit factory");
static_assert(!std::is_constructible<math::Angle, double>::value,
    "Angles must be created through a unit factory");



// 객체 생성과 상태 보존 규칙을 검증한다.
TEST(AngleTest, CreatesAnglesFromDegreeLiterals)
{
    using namespace math::literals;

    static_assert(std::is_same<decltype(30_deg), math::Angle>::value,
        "Degree literals must produce an Angle");
    EXPECT_DOUBLE_EQ((30_deg).deg(), 30.0);
    EXPECT_DOUBLE_EQ((30.5_deg).deg(), 30.5);
}

// 객체 생성과 상태 보존 규칙을 검증한다.
TEST(AngleTest, CreatesAnglesFromRadianLiterals)
{
    using namespace math::literals;

    static_assert(std::is_same<decltype(1_rad), math::Angle>::value,
        "Radian literals must produce an Angle");
    EXPECT_DOUBLE_EQ((1_rad).rad(), 1.0);
    EXPECT_DOUBLE_EQ((0.37_rad).rad(), 0.37);
}

// 값의 변환과 정규화 결과를 검증한다.
TEST(AngleTest, WrapsToSignedDegreeRange)
{
    using namespace math::literals;

    EXPECT_TRUE(math::wrap180(0_deg).equals(0_deg, math::QuantityTolerance{0.0_deg, 0.0}));
    EXPECT_TRUE(math::wrap180(179.5_deg).equals(179.5_deg, math::QuantityTolerance{0.0_deg, 0.0}));
    EXPECT_TRUE(math::wrap180(-179.5_deg).equals(-179.5_deg, math::QuantityTolerance{0.0_deg, 0.0}));
    EXPECT_TRUE(math::wrap180(180_deg).equals(-180_deg, math::QuantityTolerance{0.0_deg, 0.0}));
    EXPECT_TRUE(math::wrap180(-180_deg).equals(-180_deg, math::QuantityTolerance{0.0_deg, 0.0}));
    EXPECT_TRUE(math::wrap180(181_deg).equals(-179_deg, math::QuantityTolerance{0.0_deg, 0.0}));
    EXPECT_TRUE(math::wrap180(-181_deg).equals(179_deg, math::QuantityTolerance{0.0_deg, 0.0}));
    EXPECT_TRUE(math::wrap180(540_deg).equals(-180_deg, math::QuantityTolerance{0.0_deg, 0.0}));
    EXPECT_TRUE(math::wrap180(-540_deg).equals(-180_deg, math::QuantityTolerance{0.0_deg, 0.0}));
    EXPECT_TRUE(math::wrap180(1080_deg).equals(0_deg, math::QuantityTolerance{0.0_deg, 0.0}));
    EXPECT_TRUE(math::wrap180(-1080_deg).equals(0_deg, math::QuantityTolerance{0.0_deg, 0.0}));
    EXPECT_DOUBLE_EQ(math::wrap180(1e18_deg).deg(), -80.0);

    // 정규화는 새 각도를 반환하며 원본 값은 유지한다.
    const auto angle = 450_deg;
    EXPECT_TRUE(math::wrap180(angle).equals(90_deg, math::QuantityTolerance{0.0_deg, 0.0}));
    EXPECT_TRUE((angle).equals(450_deg, math::QuantityTolerance{0.0_deg, 0.0}));
}

// 값의 변환과 정규화 결과를 검증한다.
TEST(AngleTest, WrapsToUnsignedDegreeRange)
{
    using namespace math::literals;

    EXPECT_DOUBLE_EQ(math::wrap360(0_deg).deg(), 0.0);
    EXPECT_DOUBLE_EQ(math::wrap360(359.5_deg).deg(), 359.5);
    EXPECT_DOUBLE_EQ(math::wrap360(-0.5_deg).deg(), 359.5);
    EXPECT_DOUBLE_EQ(math::wrap360(360_deg).deg(), 0.0);
    EXPECT_DOUBLE_EQ(math::wrap360(-360_deg).deg(), 0.0);
    EXPECT_DOUBLE_EQ(math::wrap360(361_deg).deg(), 1.0);
    EXPECT_DOUBLE_EQ(math::wrap360(-361_deg).deg(), 359.0);
    EXPECT_DOUBLE_EQ(math::wrap360(720_deg).deg(), 0.0);
    EXPECT_DOUBLE_EQ(math::wrap360(-720_deg).deg(), 0.0);
    EXPECT_DOUBLE_EQ(math::wrap360(450_deg).deg(), 90.0);
    EXPECT_DOUBLE_EQ(math::wrap360(-450_deg).deg(), 270.0);
    EXPECT_DOUBLE_EQ(math::wrap360(1e18_deg).deg(), 280.0);

    const auto angle = 450_deg;
    EXPECT_DOUBLE_EQ(math::wrap360(angle).deg(), 90.0);
    EXPECT_DOUBLE_EQ(angle.deg(), 450.0);
}

// 값의 변환과 정규화 결과를 검증한다.
TEST(AngleTest, ConvertsRepresentativeDegreesToRadians)
{
    EXPECT_DOUBLE_EQ(0.0_deg.rad(), 0.0);
    EXPECT_NEAR(45.0_deg.rad(), math::PI / 4.0, 1e-15);
    EXPECT_NEAR(90.0_deg.rad(), math::PI / 2.0, 1e-15);
    EXPECT_NEAR(180.0_deg.rad(), math::PI, 1e-15);
    EXPECT_NEAR(-90.0_deg.rad(), -math::PI / 2.0, 1e-15);
    EXPECT_NEAR(-180.0_deg.rad(), -math::PI, 1e-15);

    const auto multipleTurns = -720.0_deg;
    EXPECT_DOUBLE_EQ(multipleTurns.deg(), -720.0);
    EXPECT_NEAR(multipleTurns.rad(), -4.0 * math::PI, 1e-15);
    EXPECT_DOUBLE_EQ(450.0_deg.deg(), 450.0);
}

// 값의 변환과 정규화 결과를 검증한다.
TEST(AngleTest, ConvertsRepresentativeRadiansToDegrees)
{
    EXPECT_DOUBLE_EQ(0.0_rad.deg(), 0.0);
    EXPECT_NEAR(math::Angle::fromRadians(math::PI / 4.0).deg(), 45.0, 1e-12);
    EXPECT_NEAR(math::Angle::fromRadians(math::PI / 2.0).deg(), 90.0, 1e-12);
    EXPECT_NEAR(math::Angle::fromRadians(math::PI).deg(), 180.0, 1e-12);
    EXPECT_NEAR(math::Angle::fromRadians(-math::PI / 2.0).deg(), -90.0, 1e-12);
    EXPECT_NEAR(math::Angle::fromRadians(-math::PI).deg(), -180.0, 1e-12);
    EXPECT_NEAR(math::Angle::fromRadians(4.0 * math::PI).deg(), 720.0, 1e-12);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(AngleTest, PerformsArithmeticWithMixedInputUnits)
{
    const auto quarterTurn = math::Angle::fromRadians(math::PI / 2.0);
    const auto offset = 30.0_deg;

    EXPECT_NEAR((quarterTurn + offset).deg(), 120.0, 1e-12);
    EXPECT_NEAR((offset - quarterTurn).deg(), -60.0, 1e-12);
    EXPECT_NEAR((quarterTurn * 2.0).rad(), math::PI, 1e-15);
    EXPECT_DOUBLE_EQ((2.0 * offset).deg(), 60.0);
    EXPECT_DOUBLE_EQ((offset / -2.0).deg(), -15.0);
    EXPECT_DOUBLE_EQ((+offset).deg(), 30.0);
    EXPECT_DOUBLE_EQ((-offset).deg(), -30.0);
    EXPECT_DOUBLE_EQ(offset.deg(), 30.0);

    auto accumulated = 350.0_deg;
    EXPECT_EQ(&(accumulated += offset), &accumulated);
    EXPECT_DOUBLE_EQ(accumulated.deg(), 380.0);
    EXPECT_EQ(&(accumulated -= offset), &accumulated);
    EXPECT_EQ(&(accumulated *= 2.0), &accumulated);
    EXPECT_EQ(&(accumulated /= 4.0), &accumulated);
    EXPECT_DOUBLE_EQ(accumulated.deg(), 175.0);
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(AngleTest, EqualsAcceptsToleranceCreatedFromRadians)
{
    using namespace math::literals;

    const auto tolerance = math::Angle::fromRadians(math::PI / 180.0);
    EXPECT_TRUE((0_deg).equals(0.5_deg, math::QuantityTolerance{tolerance, 0.0}));
    EXPECT_FALSE((0_deg).equals(2_deg, math::QuantityTolerance{tolerance, 0.0}));
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(AngleTest, PeriodicEquivalenceWrapsFullTurnsWhileDefaultComparisonUsesStoredDegrees)
{
    const auto zero = 0.0_deg;
    const auto negative = -10.0_deg;
    const auto fullTurn = 360.0_deg;
    const auto negativeFullTurn = -360.0_deg;
    const auto calculated = math::Angle::fromDegrees(0.1 + 0.2);
    const auto expected = 0.3_deg;

    EXPECT_FALSE(zero.equals(fullTurn, math::QuantityTolerance{0.0_deg, 0.0}));
    EXPECT_TRUE(zero.isEquivalentTo(
        fullTurn,
        math::QuantityTolerance{0.0_deg, 0.0}));
    EXPECT_TRUE(zero.isEquivalentTo(
        negativeFullTurn,
        math::QuantityTolerance{0.0_deg, 0.0}));
    EXPECT_FALSE(zero == fullTurn);
    EXPECT_FALSE(zero == negativeFullTurn);
    EXPECT_TRUE(zero.notEquals(fullTurn, math::QuantityTolerance{0.0_deg, 0.0}));
    EXPECT_TRUE(zero != fullTurn);
    EXPECT_TRUE(zero.notEquals(negative, math::QuantityTolerance{0.0_deg, 0.0}));
    EXPECT_TRUE(zero != negative);
    EXPECT_FALSE(359.75_deg.equals(
        0.25_deg,
        math::QuantityTolerance{0.5_deg, 0.0}));
    EXPECT_TRUE(359.75_deg.isEquivalentTo(
        0.25_deg,
        math::QuantityTolerance{0.5_deg, 0.0}));
    EXPECT_TRUE(calculated.equals(expected));
    EXPECT_TRUE(calculated == expected);
    EXPECT_TRUE(negative < zero);
    EXPECT_TRUE(negative <= zero);
    EXPECT_TRUE(fullTurn > zero);
    EXPECT_TRUE(fullTurn >= zero);
    EXPECT_TRUE(zero <= zero);
    EXPECT_TRUE(zero >= zero);
    EXPECT_FALSE(zero < zero);
    EXPECT_FALSE(zero > zero);
    EXPECT_FALSE(fullTurn <= zero);
    EXPECT_FALSE(negative >= zero);
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(AngleTest, EqualsUsesInclusiveAbsoluteTolerance)
{
    const auto value = -1.0_deg;
    const auto nearby = -0.75_deg;

    EXPECT_TRUE(value.equals(value, math::QuantityTolerance{0.0_deg, 0.0}));
    EXPECT_FALSE(value.equals(nearby, math::QuantityTolerance{0.0_deg, 0.0}));
    EXPECT_FALSE(value.equals(nearby, math::QuantityTolerance{0.125_deg, 0.0}));
    EXPECT_TRUE(value.equals(nearby, math::QuantityTolerance{0.25_deg, 0.0}));
    EXPECT_TRUE(nearby.equals(value, math::QuantityTolerance{0.25_deg, 0.0}));
    EXPECT_TRUE(value.equals(nearby, math::QuantityTolerance{0.5_deg, 0.0}));
    EXPECT_THROW(value.equals(value, math::QuantityTolerance{-0.25_deg, 0.0}), std::invalid_argument);
    EXPECT_THROW(value.equals(value, math::QuantityTolerance{math::Angle::fromDegrees(std::numeric_limits<double>::infinity()), 0.0}), std::invalid_argument);
    EXPECT_THROW(value.equals(value, math::QuantityTolerance{math::Angle::fromDegrees(std::numeric_limits<double>::quiet_NaN()), 0.0}), std::invalid_argument);
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(AngleTest, EqualsUsesInclusiveRelativeTolerance)
{
    const auto value = 1024.0_deg;
    const auto nearby = 896.0_deg;

    EXPECT_FALSE(value.equals(nearby, math::QuantityTolerance<math::Angle>{0.0_deg, 0.0}));
    EXPECT_TRUE(value.equals(nearby, math::QuantityTolerance<math::Angle>{0.0_deg, 0.125}));
    EXPECT_TRUE(nearby.equals(value, math::QuantityTolerance<math::Angle>{0.0_deg, 0.125}));
    EXPECT_FALSE(value.equals(nearby, math::QuantityTolerance<math::Angle>{0.0_deg, 0.124}));
    EXPECT_TRUE((-value).equals(-nearby, math::QuantityTolerance<math::Angle>{0.0_deg, 0.125}));
    EXPECT_TRUE(1.0_deg.equals(0.875_deg, math::QuantityTolerance<math::Angle>{0.0_deg, 0.125}));

    const auto zero = 0.0_deg;
    const auto tiny = 1e-10_deg;
    EXPECT_TRUE(zero.equals(zero, math::QuantityTolerance<math::Angle>{0.0_deg, 0.01}));
    EXPECT_FALSE(zero.equals(tiny, math::QuantityTolerance<math::Angle>{0.0_deg, 0.01}));
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(AngleTest, EqualsHandlesRelativeComparisonAtNumericLimits)
{
    const double largest = std::numeric_limits<double>::max();
    const auto positive = math::Angle::fromDegrees(largest);
    const auto negative = math::Angle::fromDegrees(-largest);

    EXPECT_FALSE(positive.equals(negative, math::QuantityTolerance<math::Angle>{0.0_deg, 1.5}));
    EXPECT_TRUE(positive.equals(negative, math::QuantityTolerance<math::Angle>{0.0_deg, 2.0}));
    EXPECT_TRUE(positive.equals(math::Angle::fromDegrees(largest / 2.0), math::QuantityTolerance<math::Angle>{0.0_deg, 0.5}));

    const double smallest = std::numeric_limits<double>::denorm_min();
    EXPECT_TRUE(math::Angle::fromDegrees(2.0 * smallest).equals(math::Angle::fromDegrees(smallest), math::QuantityTolerance<math::Angle>{0.0_deg, 0.5}));
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(AngleTest, AbsPreservesAngleTypeAndReturnsNonnegativeValue)
{
    using namespace math::literals;

    static_assert(std::is_same<decltype(math::abs(-30_deg)), math::Angle>::value,
        "Absolute value of an angle must remain an Angle");
    EXPECT_DOUBLE_EQ(math::abs(-30_deg).deg(), 30.0);
    EXPECT_DOUBLE_EQ(math::abs(30_deg).deg(), 30.0);
    EXPECT_DOUBLE_EQ(math::abs(0_deg).deg(), 0.0);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(AngleTest, RejectsDivisionByZeroWithoutChangingValue)
{
    auto angle = 45.0_deg;
    EXPECT_THROW(angle / 0.0, std::domain_error);
    EXPECT_THROW(angle /= -0.0, std::domain_error);
    EXPECT_DOUBLE_EQ(angle.deg(), 45.0);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(AngleTest, EqualsRejectsInvalidRelativeTolerance)
{
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();

    for (const double relativeTolerance : {-0.01, infinity, nan})
    {
        EXPECT_THROW((math::QuantityTolerance<math::Angle>{
            0.0_deg, relativeTolerance}), std::invalid_argument);
    }

}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(AngleTest, RejectsNonFiniteInputsAndConvertedValues)
{
    const double INVALID_VALUES[] =
    {
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()
    };
    for (double value : INVALID_VALUES)
    {
        EXPECT_THROW(math::Angle::fromDegrees(value), std::invalid_argument);
        EXPECT_THROW(math::Angle::fromRadians(value), std::invalid_argument);
        EXPECT_THROW(math::literals::operator""_deg(static_cast<long double>(value)), std::invalid_argument);
        EXPECT_THROW(math::literals::operator""_rad(static_cast<long double>(value)), std::invalid_argument);
    }
    const double MAX_VALUE = std::numeric_limits<double>::max();
    EXPECT_THROW(math::Angle::fromRadians(MAX_VALUE), std::invalid_argument);
    EXPECT_THROW(math::Angle::fromRadians(-MAX_VALUE), std::invalid_argument);
    EXPECT_THROW(math::literals::operator""_rad(static_cast<long double>(MAX_VALUE)), std::invalid_argument);
    EXPECT_DOUBLE_EQ(math::Angle::fromDegrees(MAX_VALUE).deg(), MAX_VALUE);
    EXPECT_DOUBLE_EQ(math::Angle::fromDegrees(-MAX_VALUE).deg(), -MAX_VALUE);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(AngleTest, RejectsArithmeticOverflowWithoutChangingValue)
{
    const double MAX_VALUE = std::numeric_limits<double>::max();
    const auto maximum = math::Angle::fromDegrees(MAX_VALUE);
    auto angle = maximum;
    EXPECT_THROW(angle + maximum, std::invalid_argument);
    EXPECT_THROW(angle - (-maximum), std::invalid_argument);
    EXPECT_THROW(angle * 2.0, std::invalid_argument);
    EXPECT_THROW(2.0 * angle, std::invalid_argument);
    EXPECT_THROW(angle / 0.5, std::invalid_argument);
    EXPECT_THROW(angle += maximum, std::invalid_argument);
    EXPECT_DOUBLE_EQ(angle.deg(), MAX_VALUE);
    EXPECT_THROW(angle -= -maximum, std::invalid_argument);
    EXPECT_DOUBLE_EQ(angle.deg(), MAX_VALUE);
    EXPECT_THROW(angle *= 2.0, std::invalid_argument);
    EXPECT_DOUBLE_EQ(angle.deg(), MAX_VALUE);
    EXPECT_THROW(angle /= 0.5, std::invalid_argument);
    EXPECT_DOUBLE_EQ(angle.deg(), MAX_VALUE);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(AngleTest, RejectsNonFiniteArithmeticResultsWithoutChangingValue)
{
    auto angle = 1.0_deg;
    const double INVALID_VALUES[] =
    {
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()
    };
    for (double value : INVALID_VALUES)
    {
        EXPECT_THROW(angle *= value, std::invalid_argument);
        EXPECT_DOUBLE_EQ(angle.deg(), 1.0);
    }
    EXPECT_THROW(angle /= std::numeric_limits<double>::quiet_NaN(), std::invalid_argument);
    EXPECT_DOUBLE_EQ(angle.deg(), 1.0);
}
