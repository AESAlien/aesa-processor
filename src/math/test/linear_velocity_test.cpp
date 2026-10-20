#include "math/quantity/linear_velocity.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <type_traits>

using namespace math::literals;

// 단위를 명시하는 팩터리를 거치도록 기본 생성과 double 직접 생성을 금지한다.
static_assert(!std::is_default_constructible<math::LinearVelocity>::value,
    "Velocities must be created through a unit factory");
static_assert(!std::is_constructible<math::LinearVelocity, double>::value,
    "Velocities must be created through a unit factory");



// 객체 생성과 상태 보존 규칙을 검증한다.
TEST(LinearVelocityTest, PreservesSignedMetersPerSecond)
{
    const double values[] = {-1234.567, -0.001, 0.0, 0.001, 1234.567, 1000000.0};

    for (const double value : values)
    {
        const auto velocity = math::LinearVelocity::fromMetersPerSecond(value);
        EXPECT_DOUBLE_EQ(velocity.mps(), value);
        EXPECT_DOUBLE_EQ((-velocity).mps(), -value);
    }
}

// 객체 생성과 상태 보존 규칙을 검증한다.
TEST(LinearVelocityTest, CreatesVelocitiesFromMetersPerSecondLiterals)
{
    using namespace math::literals;

    static_assert(std::is_same<decltype(2_mps), math::LinearVelocity>::value,
        "Integer meters per second literals must produce a LinearVelocity");
    static_assert(std::is_same<decltype(2.5_mps), math::LinearVelocity>::value,
        "Floating-point meters per second literals must produce a LinearVelocity");
    EXPECT_DOUBLE_EQ((2_mps).mps(), 2.0);
    EXPECT_DOUBLE_EQ((2.5_mps).mps(), 2.5);
}

// 객체 생성과 상태 보존 규칙을 검증한다.
TEST(LinearVelocityTest, CreatesLinearVelocityByDividingLengthByChronoDurations)
{
    using namespace math::literals;
    using namespace std::chrono_literals;

    static_assert(std::is_same<decltype(1_km / 1s), math::LinearVelocity>::value,
        "Length divided by a duration must produce a LinearVelocity");
    EXPECT_DOUBLE_EQ((1_km / 2s).mps(), 500.0);
    EXPECT_DOUBLE_EQ((1000_m / 2s).mps(), 500.0);
    EXPECT_DOUBLE_EQ((0.25_m / 500ms).mps(), 0.5);
    EXPECT_DOUBLE_EQ((36_km / 1h).mps(), 10.0);
    EXPECT_DOUBLE_EQ((1_km / 2.5s).mps(), 400.0);
    EXPECT_DOUBLE_EQ((1_km / std::chrono::duration<double, std::ratio<1, 4>>(2.0)).mps(),
        2000.0);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(LinearVelocityTest, AddsAndSubtractsVelocities)
{
    using namespace math::literals;

    static_assert(std::is_same_v<decltype(3_mps + 2_mps), math::LinearVelocity>);
    static_assert(std::is_same_v<decltype(3_mps - 2_mps), math::LinearVelocity>);
    EXPECT_EQ(3_mps + 2_mps, 5_mps);
    EXPECT_EQ(3_mps - 2_mps, 1_mps);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(LinearVelocityTest, MultipliesAndDividesByScalars)
{
    using namespace math::literals;

    static_assert(std::is_same_v<decltype(3_mps * 2.0), math::LinearVelocity>);
    static_assert(std::is_same_v<decltype(2.0 * 3_mps), math::LinearVelocity>);
    static_assert(std::is_same_v<decltype(3_mps / 2.0), math::LinearVelocity>);
    EXPECT_EQ(3_mps * 2.0, 6_mps);
    EXPECT_EQ(2.0 * 3_mps, 6_mps);
    EXPECT_EQ(3_mps / 2.0, 1.5_mps);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(LinearVelocityTest, CalculatesLengthFromVelocityAndDuration)
{
    using namespace math::literals;
    using namespace std::chrono_literals;

    static_assert(std::is_same_v<decltype(2_mps * 500ms), math::Length>);
    EXPECT_EQ(2_mps * 500ms, 1_m);
    EXPECT_EQ(-2_mps * 3s, -6_m);
    EXPECT_EQ(2_mps * -3s, -6_m);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(LinearVelocityTest, PreservesSignsWhenDividingLengthByDuration)
{
    using namespace math::literals;
    using namespace std::chrono_literals;

    EXPECT_DOUBLE_EQ((1_km / -2s).mps(), -500.0);
    EXPECT_DOUBLE_EQ((0_km / 2s).mps(), 0.0);
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(LinearVelocityTest, EqualsUsesInclusiveAbsoluteTolerance)
{
    const auto value = -1.0_mps;
    const auto nearby = -0.75_mps;

    EXPECT_TRUE(value.equals(value, math::QuantityTolerance{0.0_mps, 0.0}));
    EXPECT_FALSE(value.equals(nearby, math::QuantityTolerance{0.0_mps, 0.0}));
    EXPECT_FALSE(value.equals(nearby, math::QuantityTolerance{0.125_mps, 0.0}));
    EXPECT_TRUE(value.equals(nearby, math::QuantityTolerance{0.25_mps, 0.0}));
    EXPECT_TRUE(nearby.equals(value, math::QuantityTolerance{0.25_mps, 0.0}));
    EXPECT_TRUE(value.equals(nearby, math::QuantityTolerance{0.5_mps, 0.0}));
    EXPECT_THROW(value.equals(value, math::QuantityTolerance{-0.25_mps, 0.0}), std::invalid_argument);
    EXPECT_THROW(value.equals(value, math::QuantityTolerance{math::LinearVelocity::fromMetersPerSecond(std::numeric_limits<double>::infinity()), 0.0}), std::invalid_argument);
    EXPECT_THROW(value.equals(value, math::QuantityTolerance{math::LinearVelocity::fromMetersPerSecond(std::numeric_limits<double>::quiet_NaN()), 0.0}), std::invalid_argument);
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(LinearVelocityTest, AbsPreservesLinearVelocityTypeAndReturnsNonnegativeValue)
{
    using namespace math::literals;

    static_assert(std::is_same<decltype(math::abs(-2_mps)), math::LinearVelocity>::value,
        "Absolute value of a velocity must remain a LinearVelocity");
    EXPECT_DOUBLE_EQ(math::abs(-2_mps).mps(), 2.0);
    EXPECT_DOUBLE_EQ(math::abs(2_mps).mps(), 2.0);
    EXPECT_DOUBLE_EQ(math::abs(0_mps).mps(), 0.0);
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(LinearVelocityTest, ComparesSignedVelocities)
{
    using namespace math::literals;

    const auto zero = 0_mps;
    const auto negative = -0.5_mps;
    const auto positive = 2_mps;

    EXPECT_TRUE(negative < zero);
    EXPECT_TRUE(negative <= zero);
    EXPECT_TRUE(positive > zero);
    EXPECT_TRUE(positive >= zero);
    EXPECT_TRUE(zero <= zero);
    EXPECT_TRUE(zero >= zero);
    EXPECT_FALSE(zero < zero);
    EXPECT_FALSE(zero > zero);
    EXPECT_FALSE(positive <= zero);
    EXPECT_FALSE(negative >= zero);
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(LinearVelocityTest, EqualsUsesInclusiveRelativeTolerance)
{
    const auto value = 1024.0_mps;
    const auto nearby = 896.0_mps;

    EXPECT_FALSE(value.equals(nearby, math::QuantityTolerance<math::LinearVelocity>{0.0_mps, 0.0}));
    EXPECT_TRUE(value.equals(nearby, math::QuantityTolerance<math::LinearVelocity>{0.0_mps, 0.125}));
    EXPECT_TRUE(nearby.equals(value, math::QuantityTolerance<math::LinearVelocity>{0.0_mps, 0.125}));
    EXPECT_FALSE(value.equals(nearby, math::QuantityTolerance<math::LinearVelocity>{0.0_mps, 0.124}));
    EXPECT_TRUE((-value).equals(-nearby, math::QuantityTolerance<math::LinearVelocity>{0.0_mps, 0.125}));
    EXPECT_TRUE((1.0_mps).equals(0.875_mps, math::QuantityTolerance<math::LinearVelocity>{0.0_mps, 0.125}));

    const auto zero = 0.0_mps;
    const auto tiny = 1e-10_mps;
    EXPECT_TRUE(zero.equals(zero, math::QuantityTolerance<math::LinearVelocity>{0.0_mps, 0.01}));
    EXPECT_FALSE(zero.equals(tiny, math::QuantityTolerance<math::LinearVelocity>{0.0_mps, 0.01}));
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(LinearVelocityTest, EqualsHandlesRelativeComparisonAtNumericLimits)
{
    const double largest = std::numeric_limits<double>::max();
    const auto positive = math::LinearVelocity::fromMetersPerSecond(largest);
    const auto negative = math::LinearVelocity::fromMetersPerSecond(-largest);

    EXPECT_FALSE(positive.equals(negative, math::QuantityTolerance<math::LinearVelocity>{0.0_mps, 1.5}));
    EXPECT_TRUE(positive.equals(negative, math::QuantityTolerance<math::LinearVelocity>{0.0_mps, 2.0}));
    EXPECT_TRUE(positive.equals(math::LinearVelocity::fromMetersPerSecond(largest / 2.0), math::QuantityTolerance<math::LinearVelocity>{0.0_mps, 0.5}));

    const double smallest = std::numeric_limits<double>::denorm_min();
    EXPECT_TRUE(math::LinearVelocity::fromMetersPerSecond(2.0 * smallest).equals(math::LinearVelocity::fromMetersPerSecond(smallest), math::QuantityTolerance<math::LinearVelocity>{0.0_mps, 0.5}));
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(LinearVelocityTest, RejectsInvalidVelocityDurationProducts)
{
    using namespace math::literals;

    const auto maximum = math::LinearVelocity::fromMetersPerSecond(
        std::numeric_limits<double>::max());
    EXPECT_THROW(maximum * std::chrono::seconds{2}, std::invalid_argument);
    EXPECT_THROW(1_mps * std::chrono::duration<double>{
        std::numeric_limits<double>::quiet_NaN()}, std::invalid_argument);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(LinearVelocityTest, RejectsZeroDuration)
{
    using namespace math::literals;
    using namespace std::chrono_literals;

    EXPECT_THROW(1_km / 0s, std::domain_error);
    EXPECT_THROW(1_km / 0.0s, std::domain_error);
    EXPECT_THROW(1_km / -0.0s, std::domain_error);
    EXPECT_THROW(0_km / 0s, std::domain_error);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(LinearVelocityTest, EqualsRejectsInvalidRelativeTolerance)
{
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();

    for (const double relativeTolerance : {-0.01, infinity, nan})
    {
        EXPECT_THROW((math::QuantityTolerance<math::LinearVelocity>{
            0.0_mps, relativeTolerance}), std::invalid_argument);
    }
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(LinearVelocityTest, RejectsNonFiniteValues)
{
    const double INVALID_VALUES[] =
    {
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()
    };
    for (double value : INVALID_VALUES)
    {
        EXPECT_THROW(math::LinearVelocity::fromMetersPerSecond(value), std::invalid_argument);
        EXPECT_THROW(math::literals::operator""_mps(static_cast<long double>(value)), std::invalid_argument);
    }
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(LinearVelocityTest, RejectsNonFiniteResultsFromDurationDivision)
{
    using namespace math::literals;
    const auto length = math::Length::fromMeters(std::numeric_limits<double>::max());
    EXPECT_THROW(length / std::chrono::duration<double>(0.5), std::invalid_argument);
    EXPECT_THROW(length / std::chrono::duration<double>(-0.5), std::invalid_argument);
    EXPECT_THROW(1_m / std::chrono::duration<double>(std::numeric_limits<double>::quiet_NaN()),
        std::invalid_argument);
}
