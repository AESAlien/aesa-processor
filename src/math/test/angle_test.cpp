#include "math/quantity/angle.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <type_traits>

// 단위를 명시하는 팩터리를 거치도록 기본 생성과 double 직접 생성을 금지한다.
static_assert(!std::is_default_constructible<math::Angle>::value,
    "Angles must be created through a unit factory");
static_assert(!std::is_constructible<math::Angle, double>::value,
    "Angles must be created through a unit factory");

// 절대 차이가 허용오차 이하일 때만 같으며, 음수·무한대·NaN 허용오차와 NaN 값은 거부한다.
TEST(AngleTest, EqualsUsesInclusiveAbsoluteTolerance)
{
    const auto value = math::Angle::fromDegrees(-1.0);
    const auto nearby = math::Angle::fromDegrees(-0.75);

    EXPECT_TRUE(value.equals(value, math::QuantityTolerance{math::Angle::fromDegrees(0.0), 0.0}));
    EXPECT_FALSE(value.equals(nearby, math::QuantityTolerance{math::Angle::fromDegrees(0.0), 0.0}));
    EXPECT_FALSE(value.equals(nearby, math::QuantityTolerance{math::Angle::fromDegrees(0.125), 0.0}));
    EXPECT_TRUE(value.equals(nearby, math::QuantityTolerance{math::Angle::fromDegrees(0.25), 0.0}));
    EXPECT_TRUE(nearby.equals(value, math::QuantityTolerance{math::Angle::fromDegrees(0.25), 0.0}));
    EXPECT_TRUE(value.equals(nearby, math::QuantityTolerance{math::Angle::fromDegrees(0.5), 0.0}));
    EXPECT_THROW(value.equals(value, math::QuantityTolerance{math::Angle::fromDegrees(-0.25), 0.0}), std::invalid_argument);
    EXPECT_THROW(value.equals(value, math::QuantityTolerance{math::Angle::fromDegrees(std::numeric_limits<double>::infinity()), 0.0}), std::invalid_argument);
    EXPECT_THROW(value.equals(value, math::QuantityTolerance{math::Angle::fromDegrees(std::numeric_limits<double>::quiet_NaN()), 0.0}), std::invalid_argument);
}

// 라디안으로 만든 허용오차도 도 단위 각도 비교에 올바르게 적용한다.
TEST(AngleTest, EqualsAcceptsToleranceCreatedFromRadians)
{
    using namespace math::literals;

    const auto tolerance = math::Angle::fromRadians(math::PI / 180.0);
    EXPECT_TRUE((0_deg).equals(0.5_deg, math::QuantityTolerance{tolerance, 0.0}));
    EXPECT_FALSE((0_deg).equals(2_deg, math::QuantityTolerance{tolerance, 0.0}));
}

// 정수와 실수 도 리터럴이 올바른 Angle 값과 타입을 생성한다.
TEST(AngleTest, CreatesAnglesFromDegreeLiterals)
{
    using namespace math::literals;

    static_assert(std::is_same<decltype(30_deg), math::Angle>::value,
        "Degree literals must produce an Angle");
    EXPECT_DOUBLE_EQ((30_deg).deg(), 30.0);
    EXPECT_DOUBLE_EQ((30.5_deg).deg(), 30.5);
}

// 정수와 실수 라디안 리터럴이 올바른 Angle 값과 타입을 생성한다.
TEST(AngleTest, CreatesAnglesFromRadianLiterals)
{
    using namespace math::literals;

    static_assert(std::is_same<decltype(1_rad), math::Angle>::value,
        "Radian literals must produce an Angle");
    EXPECT_DOUBLE_EQ((1_rad).rad(), 1.0);
    EXPECT_DOUBLE_EQ((0.37_rad).rad(), 0.37);
}

// 범위 안의 값, ±180도 경계와 여러 회전 값을 [-180, 180) 범위로 정규화한다.
TEST(AngleTest, WrapsToSignedDegreeRange)
{
    using namespace math::literals;

    EXPECT_TRUE(((0_deg).wrap180()).equals(0_deg, math::QuantityTolerance{math::Angle::fromDegrees(0.0), 0.0}));
    EXPECT_TRUE(((179.5_deg).wrap180()).equals(179.5_deg, math::QuantityTolerance{math::Angle::fromDegrees(0.0), 0.0}));
    EXPECT_TRUE(((-179.5_deg).wrap180()).equals(-179.5_deg, math::QuantityTolerance{math::Angle::fromDegrees(0.0), 0.0}));
    EXPECT_TRUE(((180_deg).wrap180()).equals(-180_deg, math::QuantityTolerance{math::Angle::fromDegrees(0.0), 0.0}));
    EXPECT_TRUE(((-180_deg).wrap180()).equals(-180_deg, math::QuantityTolerance{math::Angle::fromDegrees(0.0), 0.0}));
    EXPECT_TRUE(((181_deg).wrap180()).equals(-179_deg, math::QuantityTolerance{math::Angle::fromDegrees(0.0), 0.0}));
    EXPECT_TRUE(((-181_deg).wrap180()).equals(179_deg, math::QuantityTolerance{math::Angle::fromDegrees(0.0), 0.0}));
    EXPECT_TRUE(((540_deg).wrap180()).equals(-180_deg, math::QuantityTolerance{math::Angle::fromDegrees(0.0), 0.0}));
    EXPECT_TRUE(((-540_deg).wrap180()).equals(-180_deg, math::QuantityTolerance{math::Angle::fromDegrees(0.0), 0.0}));
    EXPECT_TRUE(((1080_deg).wrap180()).equals(0_deg, math::QuantityTolerance{math::Angle::fromDegrees(0.0), 0.0}));
    EXPECT_TRUE(((-1080_deg).wrap180()).equals(0_deg, math::QuantityTolerance{math::Angle::fromDegrees(0.0), 0.0}));
    EXPECT_DOUBLE_EQ(math::Angle::fromDegrees(1e18).wrap180().deg(), -80.0);

    // 정규화는 새 각도를 반환하며 원본 값은 유지한다.
    const auto angle = 450_deg;
    EXPECT_TRUE((angle.wrap180()).equals(90_deg, math::QuantityTolerance{math::Angle::fromDegrees(0.0), 0.0}));
    EXPECT_TRUE((angle).equals(450_deg, math::QuantityTolerance{math::Angle::fromDegrees(0.0), 0.0}));
}

// 범위 안의 값, ±90도 경계와 여러 회전 값을 [-90, 90) 범위로 정규화한다.
TEST(AngleTest, WrapsToSignedNinetyDegreeRange)
{
    using namespace math::literals;

    EXPECT_DOUBLE_EQ((0_deg).wrap90().deg(), 0.0);
    EXPECT_DOUBLE_EQ((89.5_deg).wrap90().deg(), 89.5);
    EXPECT_DOUBLE_EQ((-89.5_deg).wrap90().deg(), -89.5);
    EXPECT_DOUBLE_EQ((90_deg).wrap90().deg(), -90.0);
    EXPECT_DOUBLE_EQ((-90_deg).wrap90().deg(), -90.0);
    EXPECT_DOUBLE_EQ((91_deg).wrap90().deg(), -89.0);
    EXPECT_DOUBLE_EQ((-91_deg).wrap90().deg(), 89.0);
    EXPECT_DOUBLE_EQ((180_deg).wrap90().deg(), 0.0);
    EXPECT_DOUBLE_EQ((270_deg).wrap90().deg(), -90.0);
    EXPECT_DOUBLE_EQ((-270_deg).wrap90().deg(), -90.0);
    EXPECT_DOUBLE_EQ((450_deg).wrap90().deg(), -90.0);
    EXPECT_DOUBLE_EQ((-450_deg).wrap90().deg(), -90.0);
    EXPECT_DOUBLE_EQ(math::Angle::fromDegrees(1e18).wrap90().deg(), -80.0);

    const auto angle = 450_deg;
    EXPECT_DOUBLE_EQ(angle.wrap90().deg(), -90.0);
    EXPECT_DOUBLE_EQ(angle.deg(), 450.0);
}

// 0과 양수·음수 각도의 도→라디안 변환을 검증하고, 회전 범위를 넘는 도 값도 그대로 보존한다.
TEST(AngleTest, ConvertsRepresentativeDegreesToRadians)
{
    EXPECT_DOUBLE_EQ(math::Angle::fromDegrees(0.0).rad(), 0.0);
    EXPECT_NEAR(math::Angle::fromDegrees(45.0).rad(), math::PI / 4.0, 1e-15);
    EXPECT_NEAR(math::Angle::fromDegrees(90.0).rad(), math::PI / 2.0, 1e-15);
    EXPECT_NEAR(math::Angle::fromDegrees(180.0).rad(), math::PI, 1e-15);
    EXPECT_NEAR(math::Angle::fromDegrees(-90.0).rad(), -math::PI / 2.0, 1e-15);
    EXPECT_NEAR(math::Angle::fromDegrees(-180.0).rad(), -math::PI, 1e-15);

    const auto multipleTurns = math::Angle::fromDegrees(-720.0);
    EXPECT_DOUBLE_EQ(multipleTurns.deg(), -720.0);
    EXPECT_NEAR(multipleTurns.rad(), -4.0 * math::PI, 1e-15);
    EXPECT_DOUBLE_EQ(math::Angle::fromDegrees(450.0).deg(), 450.0);
}

// 0과 양수·음수 각도의 라디안→도 변환을 검증하고, 회전 범위를 넘는 값도 자동 정규화하지 않는다.
TEST(AngleTest, ConvertsRepresentativeRadiansToDegrees)
{
    EXPECT_DOUBLE_EQ(math::Angle::fromRadians(0.0).deg(), 0.0);
    EXPECT_NEAR(math::Angle::fromRadians(math::PI / 4.0).deg(), 45.0, 1e-12);
    EXPECT_NEAR(math::Angle::fromRadians(math::PI / 2.0).deg(), 90.0, 1e-12);
    EXPECT_NEAR(math::Angle::fromRadians(math::PI).deg(), 180.0, 1e-12);
    EXPECT_NEAR(math::Angle::fromRadians(-math::PI / 2.0).deg(), -90.0, 1e-12);
    EXPECT_NEAR(math::Angle::fromRadians(-math::PI).deg(), -180.0, 1e-12);
    EXPECT_NEAR(math::Angle::fromRadians(4.0 * math::PI).deg(), 720.0, 1e-12);
}

// 서로 다른 단위로 만든 각도의 산술 연산과 복합 대입 결과 및 자기 참조 반환을 검증한다.
TEST(AngleTest, PerformsArithmeticWithMixedInputUnits)
{
    const auto quarterTurn = math::Angle::fromRadians(math::PI / 2.0);
    const auto offset = math::Angle::fromDegrees(30.0);

    EXPECT_NEAR((quarterTurn + offset).deg(), 120.0, 1e-12);
    EXPECT_NEAR((offset - quarterTurn).deg(), -60.0, 1e-12);
    EXPECT_NEAR((quarterTurn * 2.0).rad(), math::PI, 1e-15);
    EXPECT_DOUBLE_EQ((2.0 * offset).deg(), 60.0);
    EXPECT_DOUBLE_EQ((offset / -2.0).deg(), -15.0);
    EXPECT_DOUBLE_EQ((+offset).deg(), 30.0);
    EXPECT_DOUBLE_EQ((-offset).deg(), -30.0);
    EXPECT_DOUBLE_EQ(offset.deg(), 30.0);

    auto accumulated = math::Angle::fromDegrees(350.0);
    EXPECT_EQ(&(accumulated += offset), &accumulated);
    EXPECT_DOUBLE_EQ(accumulated.deg(), 380.0);
    EXPECT_EQ(&(accumulated -= offset), &accumulated);
    EXPECT_EQ(&(accumulated *= 2.0), &accumulated);
    EXPECT_EQ(&(accumulated /= 4.0), &accumulated);
    EXPECT_DOUBLE_EQ(accumulated.deg(), 175.0);
}

// 0으로 나누면 예외가 발생하고, 실패한 복합 대입은 원본 각도를 바꾸지 않는다.
TEST(AngleTest, RejectsDivisionByZeroWithoutChangingValue)
{
    auto angle = math::Angle::fromDegrees(45.0);
    EXPECT_THROW(angle / 0.0, std::domain_error);
    EXPECT_THROW(angle /= -0.0, std::domain_error);
    EXPECT_DOUBLE_EQ(angle.deg(), 45.0);
}

// 각도는 회전 수를 포함한 저장값으로 비교하며, 대소 비교에서 같은 값의 경계를 구분한다.
TEST(AngleTest, ComparesStoredDegreesWithoutWrapping)
{
    const auto zero = math::Angle::fromDegrees(0.0);
    const auto negative = math::Angle::fromDegrees(-10.0);
    const auto fullTurn = math::Angle::fromDegrees(360.0);

    EXPECT_FALSE(zero.equals(fullTurn, math::QuantityTolerance{math::Angle::fromDegrees(0.0), 0.0}));
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

// 상대 오차는 큰 쪽 값의 크기를 기준으로 대칭 비교하며, 허용오차 경계를 포함한다.
TEST(AngleTest, EqualsUsesInclusiveRelativeTolerance)
{
    const auto value = math::Angle::fromDegrees(1024.0);
    const auto nearby = math::Angle::fromDegrees(896.0);

    EXPECT_FALSE(value.equals(nearby, math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 0.0}));
    EXPECT_TRUE(value.equals(nearby, math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 0.125}));
    EXPECT_TRUE(nearby.equals(value, math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 0.125}));
    EXPECT_FALSE(value.equals(nearby, math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 0.124}));
    EXPECT_TRUE((-value).equals(-nearby, math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 0.125}));
    EXPECT_TRUE(math::Angle::fromDegrees(1.0).equals(math::Angle::fromDegrees(0.875), math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 0.125}));

    const auto zero = math::Angle::fromDegrees(0.0);
    const auto tiny = math::Angle::fromDegrees(1e-10);
    EXPECT_TRUE(zero.equals(zero, math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 0.01}));
    EXPECT_FALSE(zero.equals(tiny, math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 0.01}));
}

// 같은 값끼리 비교해도 잘못된 상대 허용오차는 거부하며, 유한하지 않은 값도 거부한다.
TEST(AngleTest, EqualsRejectsInvalidRelativeTolerance)
{
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();

    for (const double relativeTolerance : {-0.01, infinity, nan})
    {
        EXPECT_THROW((math::QuantityTolerance<math::Angle>{
            math::Angle::fromDegrees(0.0), relativeTolerance}), std::invalid_argument);
    }

}

// 큰 값의 차나 허용오차의 곱이 오버플로하거나 매우 작은 값을 비교해도 상대 오차를 올바르게 판단한다.
TEST(AngleTest, EqualsHandlesRelativeComparisonAtNumericLimits)
{
    const double largest = std::numeric_limits<double>::max();
    const auto positive = math::Angle::fromDegrees(largest);
    const auto negative = math::Angle::fromDegrees(-largest);

    EXPECT_FALSE(positive.equals(negative, math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 1.5}));
    EXPECT_TRUE(positive.equals(negative, math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 2.0}));
    EXPECT_TRUE(positive.equals(math::Angle::fromDegrees(largest / 2.0), math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 0.5}));

    const double smallest = std::numeric_limits<double>::denorm_min();
    EXPECT_TRUE(math::Angle::fromDegrees(2.0 * smallest).equals(math::Angle::fromDegrees(smallest), math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 0.5}));
}

TEST(AngleTest, AbsPreservesAngleTypeAndReturnsNonnegativeValue)
{
    using namespace math::literals;

    static_assert(std::is_same<decltype(math::abs(-30_deg)), math::Angle>::value,
        "Absolute value of an angle must remain an Angle");
    EXPECT_DOUBLE_EQ(math::abs(-30_deg).deg(), 30.0);
    EXPECT_DOUBLE_EQ(math::abs(30_deg).deg(), 30.0);
    EXPECT_DOUBLE_EQ(math::abs(0_deg).deg(), 0.0);
}

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

TEST(AngleTest, RejectsNonFiniteArithmeticResultsWithoutChangingValue)
{
    auto angle = math::Angle::fromDegrees(1.0);
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
