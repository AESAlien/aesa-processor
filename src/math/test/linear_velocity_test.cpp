#include "math/quantity/linear_velocity.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <type_traits>

// 단위를 명시하는 팩터리를 거치도록 기본 생성과 double 직접 생성을 금지한다.
static_assert(!std::is_default_constructible<math::LinearVelocity>::value,
    "Velocities must be created through a unit factory");
static_assert(!std::is_constructible<math::LinearVelocity, double>::value,
    "Velocities must be created through a unit factory");

// 절대 차이가 허용오차 이하일 때만 같으며, 음수·무한대·NaN 허용오차와 NaN 값은 거부한다.
TEST(LinearVelocityTest, EqualsUsesInclusiveAbsoluteTolerance)
{
    const auto value = math::LinearVelocity::fromMetersPerSecond(-1.0);
    const auto nearby = math::LinearVelocity::fromMetersPerSecond(-0.75);

    EXPECT_TRUE(value.equals(value, math::QuantityTolerance{math::LinearVelocity::fromMetersPerSecond(0.0), 0.0}));
    EXPECT_FALSE(value.equals(nearby, math::QuantityTolerance{math::LinearVelocity::fromMetersPerSecond(0.0), 0.0}));
    EXPECT_FALSE(value.equals(nearby, math::QuantityTolerance{math::LinearVelocity::fromMetersPerSecond(0.125), 0.0}));
    EXPECT_TRUE(value.equals(nearby, math::QuantityTolerance{math::LinearVelocity::fromMetersPerSecond(0.25), 0.0}));
    EXPECT_TRUE(nearby.equals(value, math::QuantityTolerance{math::LinearVelocity::fromMetersPerSecond(0.25), 0.0}));
    EXPECT_TRUE(value.equals(nearby, math::QuantityTolerance{math::LinearVelocity::fromMetersPerSecond(0.5), 0.0}));
    EXPECT_THROW(value.equals(value, math::QuantityTolerance{math::LinearVelocity::fromMetersPerSecond(-0.25), 0.0}), std::invalid_argument);
    EXPECT_THROW(value.equals(value, math::QuantityTolerance{math::LinearVelocity::fromMetersPerSecond(std::numeric_limits<double>::infinity()), 0.0}), std::invalid_argument);
    EXPECT_THROW(value.equals(value, math::QuantityTolerance{math::LinearVelocity::fromMetersPerSecond(std::numeric_limits<double>::quiet_NaN()), 0.0}), std::invalid_argument);
}

// 초당 미터로 생성한 속도의 부호와 값을 보존하고 부호 반전을 검증한다.
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

TEST(LinearVelocityTest, AddsAndSubtractsVelocities)
{
    using namespace math::literals;

    static_assert(std::is_same_v<decltype(3_mps + 2_mps), math::LinearVelocity>);
    static_assert(std::is_same_v<decltype(3_mps - 2_mps), math::LinearVelocity>);
    EXPECT_EQ(3_mps + 2_mps, 5_mps);
    EXPECT_EQ(3_mps - 2_mps, 1_mps);
}

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

TEST(LinearVelocityTest, CalculatesLengthFromVelocityAndDuration)
{
    using namespace math::literals;
    using namespace std::chrono_literals;

    static_assert(std::is_same_v<decltype(2_mps * 500ms), math::Length>);
    EXPECT_EQ(2_mps * 500ms, 1_m);
    EXPECT_EQ(-2_mps * 3s, -6_m);
    EXPECT_EQ(2_mps * -3s, -6_m);
}

TEST(LinearVelocityTest, RejectsInvalidVelocityDurationProducts)
{
    using namespace math::literals;

    const auto maximum = math::LinearVelocity::fromMetersPerSecond(
        std::numeric_limits<double>::max());
    EXPECT_THROW(maximum * std::chrono::seconds{2}, std::invalid_argument);
    EXPECT_THROW(1_mps * std::chrono::duration<double>{
        std::numeric_limits<double>::quiet_NaN()}, std::invalid_argument);
}


TEST(LinearVelocityTest, AbsPreservesLinearVelocityTypeAndReturnsNonnegativeValue)
{
    using namespace math::literals;

    static_assert(std::is_same<decltype(math::abs(-2_mps)), math::LinearVelocity>::value,
        "Absolute value of a velocity must remain a LinearVelocity");
    EXPECT_DOUBLE_EQ(math::abs(-2_mps).mps(), 2.0);
    EXPECT_DOUBLE_EQ(math::abs(2_mps).mps(), 2.0);
    EXPECT_DOUBLE_EQ(math::abs(0_mps).mps(), 0.0);
}

// 정수와 실수 초당 미터 리터럴이 올바른 LinearVelocity 값과 타입을 생성한다.
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

// 음수·0·양수 속도의 대소 비교와 같은 값에 대한 엄격·비엄격 비교를 검증한다.
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

// 좌표값과 정수·실수 시간, 사용자 정의 시간 비율을 초당 미터 속도로 변환한다.
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

// 좌표값과 시간의 부호 조합을 속도에 반영하고 좌표값 0은 속도 0으로 계산한다.
TEST(LinearVelocityTest, PreservesSignsWhenDividingLengthByDuration)
{
    using namespace math::literals;
    using namespace std::chrono_literals;

    EXPECT_DOUBLE_EQ((1_km / -2s).mps(), -500.0);
    EXPECT_DOUBLE_EQ((0_km / 2s).mps(), 0.0);
}

// 정수·실수의 0 시간과 음의 0 시간으로 나누면 거리 값에 관계없이 예외를 발생시킨다.
TEST(LinearVelocityTest, RejectsZeroDuration)
{
    using namespace math::literals;
    using namespace std::chrono_literals;

    EXPECT_THROW(1_km / 0s, std::domain_error);
    EXPECT_THROW(1_km / 0.0s, std::domain_error);
    EXPECT_THROW(1_km / -0.0s, std::domain_error);
    EXPECT_THROW(0_km / 0s, std::domain_error);
}

// 상대 오차는 큰 쪽 값의 크기를 기준으로 대칭 비교하며, 허용오차 경계를 포함한다.
TEST(LinearVelocityTest, EqualsUsesInclusiveRelativeTolerance)
{
    const auto value = math::LinearVelocity::fromMetersPerSecond(1024.0);
    const auto nearby = math::LinearVelocity::fromMetersPerSecond(896.0);

    EXPECT_FALSE(value.equals(nearby, math::QuantityTolerance<math::LinearVelocity>{math::LinearVelocity::fromMetersPerSecond(0.0), 0.0}));
    EXPECT_TRUE(value.equals(nearby, math::QuantityTolerance<math::LinearVelocity>{math::LinearVelocity::fromMetersPerSecond(0.0), 0.125}));
    EXPECT_TRUE(nearby.equals(value, math::QuantityTolerance<math::LinearVelocity>{math::LinearVelocity::fromMetersPerSecond(0.0), 0.125}));
    EXPECT_FALSE(value.equals(nearby, math::QuantityTolerance<math::LinearVelocity>{math::LinearVelocity::fromMetersPerSecond(0.0), 0.124}));
    EXPECT_TRUE((-value).equals(-nearby, math::QuantityTolerance<math::LinearVelocity>{math::LinearVelocity::fromMetersPerSecond(0.0), 0.125}));
    EXPECT_TRUE(math::LinearVelocity::fromMetersPerSecond(1.0).equals(math::LinearVelocity::fromMetersPerSecond(0.875), math::QuantityTolerance<math::LinearVelocity>{math::LinearVelocity::fromMetersPerSecond(0.0), 0.125}));

    const auto zero = math::LinearVelocity::fromMetersPerSecond(0.0);
    const auto tiny = math::LinearVelocity::fromMetersPerSecond(1e-10);
    EXPECT_TRUE(zero.equals(zero, math::QuantityTolerance<math::LinearVelocity>{math::LinearVelocity::fromMetersPerSecond(0.0), 0.01}));
    EXPECT_FALSE(zero.equals(tiny, math::QuantityTolerance<math::LinearVelocity>{math::LinearVelocity::fromMetersPerSecond(0.0), 0.01}));
}

// 같은 값끼리 비교해도 잘못된 상대 허용오차는 거부하며, 유한하지 않은 값도 거부한다.
TEST(LinearVelocityTest, EqualsRejectsInvalidRelativeTolerance)
{
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();

    for (const double relativeTolerance : {-0.01, infinity, nan})
    {
        EXPECT_THROW((math::QuantityTolerance<math::LinearVelocity>{
            math::LinearVelocity::fromMetersPerSecond(0.0), relativeTolerance}), std::invalid_argument);
    }
}

// 큰 값의 차나 허용오차의 곱이 오버플로하거나 매우 작은 값을 비교해도 상대 오차를 올바르게 판단한다.
TEST(LinearVelocityTest, EqualsHandlesRelativeComparisonAtNumericLimits)
{
    const double largest = std::numeric_limits<double>::max();
    const auto positive = math::LinearVelocity::fromMetersPerSecond(largest);
    const auto negative = math::LinearVelocity::fromMetersPerSecond(-largest);

    EXPECT_FALSE(positive.equals(negative, math::QuantityTolerance<math::LinearVelocity>{math::LinearVelocity::fromMetersPerSecond(0.0), 1.5}));
    EXPECT_TRUE(positive.equals(negative, math::QuantityTolerance<math::LinearVelocity>{math::LinearVelocity::fromMetersPerSecond(0.0), 2.0}));
    EXPECT_TRUE(positive.equals(math::LinearVelocity::fromMetersPerSecond(largest / 2.0), math::QuantityTolerance<math::LinearVelocity>{math::LinearVelocity::fromMetersPerSecond(0.0), 0.5}));

    const double smallest = std::numeric_limits<double>::denorm_min();
    EXPECT_TRUE(math::LinearVelocity::fromMetersPerSecond(2.0 * smallest).equals(math::LinearVelocity::fromMetersPerSecond(smallest), math::QuantityTolerance<math::LinearVelocity>{math::LinearVelocity::fromMetersPerSecond(0.0), 0.5}));
}

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

TEST(LinearVelocityTest, RejectsNonFiniteResultsFromDurationDivision)
{
    using namespace math::literals;
    const auto length = math::Length::fromMeters(std::numeric_limits<double>::max());
    EXPECT_THROW(length / std::chrono::duration<double>(0.5), std::invalid_argument);
    EXPECT_THROW(length / std::chrono::duration<double>(-0.5), std::invalid_argument);
    EXPECT_THROW(1_m / std::chrono::duration<double>(std::numeric_limits<double>::quiet_NaN()),
        std::invalid_argument);
}
