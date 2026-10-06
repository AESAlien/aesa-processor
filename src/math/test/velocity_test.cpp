#include "math/velocity.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <type_traits>

// 단위를 명시하는 팩터리를 거치도록 기본 생성과 double 직접 생성을 금지한다.
static_assert(!std::is_default_constructible<math::Velocity>::value,
    "Velocities must be created through a unit factory");
static_assert(!std::is_constructible<math::Velocity, double>::value,
    "Velocities must be created through a unit factory");

// 절대 차이가 허용오차 이하일 때만 같으며, 음수·무한대·NaN 허용오차와 NaN 값은 거부한다.
TEST(VelocityTest, EqualsUsesInclusiveAbsoluteTolerance)
{
    const auto value = math::Velocity::fromMetersPerSecond(-1.0);
    const auto nearby = math::Velocity::fromMetersPerSecond(-0.75);

    EXPECT_TRUE(value.equals(value, math::Velocity::fromMetersPerSecond(0.0)));
    EXPECT_FALSE(value.equals(nearby, math::Velocity::fromMetersPerSecond(0.0)));
    EXPECT_FALSE(value.equals(nearby, math::Velocity::fromMetersPerSecond(0.125)));
    EXPECT_TRUE(value.equals(nearby, math::Velocity::fromMetersPerSecond(0.25)));
    EXPECT_TRUE(nearby.equals(value, math::Velocity::fromMetersPerSecond(0.25)));
    EXPECT_TRUE(value.equals(nearby, math::Velocity::fromMetersPerSecond(0.5)));
    EXPECT_FALSE(value.equals(value, math::Velocity::fromMetersPerSecond(-0.25)));
    EXPECT_FALSE(value.equals(value, math::Velocity::fromMetersPerSecond(std::numeric_limits<double>::infinity())));
    EXPECT_FALSE(value.equals(value, math::Velocity::fromMetersPerSecond(std::numeric_limits<double>::quiet_NaN())));
    EXPECT_FALSE(value.equals(math::Velocity::fromMetersPerSecond(
        std::numeric_limits<double>::quiet_NaN()), math::Velocity::fromMetersPerSecond(0.25)));
}

// 초당 미터로 생성한 속도의 부호와 값을 보존하고 부호 반전을 검증한다.
TEST(VelocityTest, PreservesSignedMetersPerSecond)
{
    const double values[] = {-1234.567, -0.001, 0.0, 0.001, 1234.567, 1000000.0};

    for (const double value : values)
    {
        const auto velocity = math::Velocity::fromMetersPerSecond(value);
        EXPECT_DOUBLE_EQ(velocity.mps(), value);
        EXPECT_DOUBLE_EQ((-velocity).mps(), -value);
    }
}

// 정수와 실수 초당 미터 리터럴이 올바른 Velocity 값과 타입을 생성한다.
TEST(VelocityTest, CreatesVelocitiesFromMetersPerSecondLiterals)
{
    using namespace math::literals;

    static_assert(std::is_same<decltype(2_mps), math::Velocity>::value,
        "Integer meters per second literals must produce a Velocity");
    static_assert(std::is_same<decltype(2.5_mps), math::Velocity>::value,
        "Floating-point meters per second literals must produce a Velocity");
    EXPECT_DOUBLE_EQ((2_mps).mps(), 2.0);
    EXPECT_DOUBLE_EQ((2.5_mps).mps(), 2.5);
}

// 음수·0·양수 속도의 대소 비교와 같은 값에 대한 엄격·비엄격 비교를 검증한다.
TEST(VelocityTest, ComparesSignedVelocities)
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

// 거리와 정수·실수 시간, 사용자 정의 시간 비율을 초당 미터 속도로 변환한다.
TEST(VelocityTest, CreatesVelocityByDividingDistanceByChronoDurations)
{
    using namespace math::literals;
    using namespace std::chrono_literals;

    static_assert(std::is_same<decltype(1_km / 1s), math::Velocity>::value,
        "Distance divided by a duration must produce a Velocity");
    EXPECT_DOUBLE_EQ((1_km / 2s).mps(), 500.0);
    EXPECT_DOUBLE_EQ((1000_m / 2s).mps(), 500.0);
    EXPECT_DOUBLE_EQ((0.25_m / 500ms).mps(), 0.5);
    EXPECT_DOUBLE_EQ((36_km / 1h).mps(), 10.0);
    EXPECT_DOUBLE_EQ((1_km / 2.5s).mps(), 400.0);
    EXPECT_DOUBLE_EQ((1_km / std::chrono::duration<double, std::ratio<1, 4>>(2.0)).mps(),
        2000.0);
}

// 거리와 시간의 부호 조합을 속도에 반영하고 거리 0은 속도 0으로 계산한다.
TEST(VelocityTest, PreservesSignsWhenDividingDistanceByDuration)
{
    using namespace math::literals;
    using namespace std::chrono_literals;

    EXPECT_DOUBLE_EQ((-1_km / 2s).mps(), -500.0);
    EXPECT_DOUBLE_EQ((1_km / -2s).mps(), -500.0);
    EXPECT_DOUBLE_EQ((-1_km / -2s).mps(), 500.0);
    EXPECT_DOUBLE_EQ((0_km / 2s).mps(), 0.0);
}

// 정수·실수의 0 시간과 음의 0 시간으로 나누면 거리 값에 관계없이 예외를 발생시킨다.
TEST(VelocityTest, RejectsZeroDuration)
{
    using namespace math::literals;
    using namespace std::chrono_literals;

    EXPECT_THROW(1_km / 0s, std::domain_error);
    EXPECT_THROW(1_km / 0.0s, std::domain_error);
    EXPECT_THROW(1_km / -0.0s, std::domain_error);
    EXPECT_THROW(0_km / 0s, std::domain_error);
}

// 상대 오차는 큰 쪽 값의 크기를 기준으로 대칭 비교하며, 허용오차 경계를 포함한다.
TEST(VelocityTest, EqualsUsesInclusiveRelativeTolerance)
{
    const auto value = math::Velocity::fromMetersPerSecond(1024.0);
    const auto nearby = math::Velocity::fromMetersPerSecond(896.0);

    EXPECT_FALSE(value.equals(nearby, 0.0));
    EXPECT_TRUE(value.equals(nearby, 0.125));
    EXPECT_TRUE(nearby.equals(value, 0.125));
    EXPECT_FALSE(value.equals(nearby, 0.124));
    EXPECT_TRUE((-value).equals(-nearby, 0.125));
    EXPECT_TRUE(math::Velocity::fromMetersPerSecond(1.0).equals(
        math::Velocity::fromMetersPerSecond(0.875), 0.125));

    const auto zero = math::Velocity::fromMetersPerSecond(0.0);
    const auto tiny = math::Velocity::fromMetersPerSecond(1e-10);
    EXPECT_TRUE(zero.equals(zero, 0.01));
    EXPECT_FALSE(zero.equals(tiny, 0.01));
}

// 같은 값끼리 비교해도 잘못된 상대 허용오차는 거부하며, 유한하지 않은 값도 거부한다.
TEST(VelocityTest, EqualsRejectsInvalidRelativeToleranceAndNonfiniteValues)
{
    const auto value = math::Velocity::fromMetersPerSecond(1.0);
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();

    for (const double relativeTolerance : {-0.01, infinity, nan})
    {
        EXPECT_FALSE(value.equals(value, relativeTolerance));
    }
    for (const double nonfinite : {infinity, -infinity, nan})
    {
        const auto invalid = math::Velocity::fromMetersPerSecond(nonfinite);
        EXPECT_FALSE(value.equals(invalid, 0.01));
        EXPECT_FALSE(invalid.equals(value, 0.01));
        EXPECT_FALSE(invalid.equals(invalid, 0.01));
    }
}

// 큰 값의 차나 허용오차의 곱이 오버플로하거나 매우 작은 값을 비교해도 상대 오차를 올바르게 판단한다.
TEST(VelocityTest, EqualsHandlesRelativeComparisonAtNumericLimits)
{
    const double largest = std::numeric_limits<double>::max();
    const auto positive = math::Velocity::fromMetersPerSecond(largest);
    const auto negative = math::Velocity::fromMetersPerSecond(-largest);

    EXPECT_FALSE(positive.equals(negative, 1.5));
    EXPECT_TRUE(positive.equals(negative, 2.0));
    EXPECT_TRUE(positive.equals(math::Velocity::fromMetersPerSecond(largest / 2.0), 0.5));

    const double smallest = std::numeric_limits<double>::denorm_min();
    EXPECT_TRUE(math::Velocity::fromMetersPerSecond(2.0 * smallest).equals(
        math::Velocity::fromMetersPerSecond(smallest), 0.5));
}
