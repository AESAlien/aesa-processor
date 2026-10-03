#include "math/distance.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <type_traits>

// 단위를 명시하는 팩터리를 거치도록 기본 생성과 double 직접 생성을 금지한다.
static_assert(!std::is_default_constructible<math::Distance>::value,
    "Distances must be created through a unit factory");
static_assert(!std::is_constructible<math::Distance, double>::value,
    "Distances must be created through a unit factory");

// 절대 차이가 허용오차 이하일 때만 같으며, 음수·무한대·NaN 허용오차와 NaN 값은 거부한다.
TEST(DistanceTest, EqualsUsesInclusiveAbsoluteTolerance)
{
    const auto value = math::Distance::fromKilometers(-1.0);
    const auto nearby = math::Distance::fromKilometers(-0.75);

    EXPECT_TRUE(value.equals(value, math::Distance::fromKilometers(0.0)));
    EXPECT_FALSE(value.equals(nearby, math::Distance::fromKilometers(0.0)));
    EXPECT_FALSE(value.equals(nearby, math::Distance::fromKilometers(0.125)));
    EXPECT_TRUE(value.equals(nearby, math::Distance::fromKilometers(0.25)));
    EXPECT_TRUE(nearby.equals(value, math::Distance::fromKilometers(0.25)));
    EXPECT_TRUE(value.equals(nearby, math::Distance::fromKilometers(0.5)));
    EXPECT_FALSE(value.equals(value, math::Distance::fromKilometers(-0.25)));
    EXPECT_FALSE(value.equals(value, math::Distance::fromKilometers(std::numeric_limits<double>::infinity())));
    EXPECT_FALSE(value.equals(value, math::Distance::fromKilometers(std::numeric_limits<double>::quiet_NaN())));
    EXPECT_FALSE(value.equals(math::Distance::fromKilometers(
        std::numeric_limits<double>::quiet_NaN()), math::Distance::fromKilometers(0.25)));
}

// 미터로 생성한 거리의 부호와 값을 보존하고, 킬로미터 조회 및 부호 반전을 검증한다.
TEST(DistanceTest, StoresMetersWithoutChangingValue)
{
    const double values[] = {-1234.567, -0.001, 0.0, 0.001, 1234.567, 1000000.0};

    for (const double value : values)
    {
        const auto distance = math::Distance::fromMeters(value);
        EXPECT_DOUBLE_EQ(distance.m(), value);
        EXPECT_DOUBLE_EQ(distance.km(), value / 1000.0);
        EXPECT_DOUBLE_EQ((-distance).m(), -value);
    }
}

// 킬로미터 입력을 미터로 변환하며 음수·0·소수·큰 값도 올바르게 처리한다.
TEST(DistanceTest, ConvertsKilometersToStoredMeters)
{
    const double values[] = {-1234.567, -0.001, 0.0, 0.001, 1234.567, 1000000.0};

    for (const double value : values)
    {
        const auto distance = math::Distance::fromKilometers(value);
        EXPECT_DOUBLE_EQ(distance.m(), value * 1000.0);
        EXPECT_DOUBLE_EQ(distance.km(), value);
    }
}

// 정수와 실수 킬로미터 리터럴이 올바른 Distance 값과 타입을 생성한다.
TEST(DistanceTest, CreatesDistancesFromKilometerLiterals)
{
    using namespace math::literals;

    static_assert(std::is_same<decltype(2_km), math::Distance>::value,
        "Integer kilometer literals must produce a Distance");
    static_assert(std::is_same<decltype(2.5_km), math::Distance>::value,
        "Floating-point kilometer literals must produce a Distance");
    EXPECT_DOUBLE_EQ(2_km.km(), 2.0);
    EXPECT_DOUBLE_EQ(2.5_km.km(), 2.5);
}

// 정수와 실수 미터 리터럴이 올바른 Distance 값과 타입을 생성한다.
TEST(DistanceTest, CreatesDistancesFromMeterLiterals)
{
    using namespace math::literals;

    static_assert(std::is_same<decltype(2_m), math::Distance>::value,
        "Integer meter literals must produce a Distance");
    static_assert(std::is_same<decltype(2.5_m), math::Distance>::value,
        "Floating-point meter literals must produce a Distance");
    EXPECT_DOUBLE_EQ(2_m.m(), 2.0);
    EXPECT_DOUBLE_EQ(2.5_m.m(), 2.5);
}

// 미터와 킬로미터를 혼용해도 값과 허용오차를 같은 단위로 환산하여 비교한다.
TEST(DistanceTest, ComparesMixedUnitsWithMeterTolerance)
{
    using namespace math::literals;

    EXPECT_TRUE(1_km.equals(1000_m, 0_m));
    EXPECT_TRUE(1_km.equals(1000.25_m, 0.25_m));
    EXPECT_FALSE(1_km.equals(1000.25_m, 0.125_m));
    EXPECT_TRUE(1000_m.equals(1.125_km, 0.125_km));
    EXPECT_TRUE(999_m < 1_km);
    EXPECT_TRUE(1001_m > 1_km);
    EXPECT_TRUE(1000_m <= 1_km);
    EXPECT_TRUE(1000_m >= 1_km);
}

// 음수·0·양수 거리의 대소 비교와 같은 값에 대한 엄격·비엄격 비교를 검증한다.
TEST(DistanceTest, ComparesStoredMeters)
{
    using namespace math::literals;

    const auto zero = 0_km;
    const auto negative = -0.5_km;
    const auto positive = 2_km;

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

// 상대 오차는 큰 쪽 값의 크기를 기준으로 대칭 비교하며, 허용오차 경계를 포함한다.
TEST(DistanceTest, EqualsUsesInclusiveRelativeTolerance)
{
    const auto value = math::Distance::fromMeters(1024.0);
    const auto nearby = math::Distance::fromMeters(896.0);

    EXPECT_FALSE(value.equals(nearby, 0.0));
    EXPECT_TRUE(value.equals(nearby, 0.125));
    EXPECT_TRUE(nearby.equals(value, 0.125));
    EXPECT_FALSE(value.equals(nearby, 0.124));
    EXPECT_TRUE((-value).equals(-nearby, 0.125));
    EXPECT_TRUE(math::Distance::fromMeters(1.0).equals(
        math::Distance::fromMeters(0.875), 0.125));

    const auto zero = math::Distance::fromMeters(0.0);
    const auto tiny = math::Distance::fromMeters(1e-10);
    EXPECT_TRUE(zero.equals(zero, 0.01));
    EXPECT_FALSE(zero.equals(tiny, 0.01));
}

// 같은 값끼리 비교해도 잘못된 상대 허용오차는 거부하며, 유한하지 않은 값도 거부한다.
TEST(DistanceTest, EqualsRejectsInvalidRelativeToleranceAndNonfiniteValues)
{
    const auto value = math::Distance::fromMeters(1.0);
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();

    for (const double relativeTolerance : {-0.01, infinity, nan})
    {
        EXPECT_FALSE(value.equals(value, relativeTolerance));
    }
    for (const double nonfinite : {infinity, -infinity, nan})
    {
        const auto invalid = math::Distance::fromMeters(nonfinite);
        EXPECT_FALSE(value.equals(invalid, 0.01));
        EXPECT_FALSE(invalid.equals(value, 0.01));
        EXPECT_FALSE(invalid.equals(invalid, 0.01));
    }
}

// 큰 값의 차나 허용오차의 곱이 오버플로하거나 매우 작은 값을 비교해도 상대 오차를 올바르게 판단한다.
TEST(DistanceTest, EqualsHandlesRelativeComparisonAtNumericLimits)
{
    const double largest = std::numeric_limits<double>::max();
    const auto positive = math::Distance::fromMeters(largest);
    const auto negative = math::Distance::fromMeters(-largest);

    EXPECT_FALSE(positive.equals(negative, 1.5));
    EXPECT_TRUE(positive.equals(negative, 2.0));
    EXPECT_TRUE(positive.equals(math::Distance::fromMeters(largest / 2.0), 0.5));

    const double smallest = std::numeric_limits<double>::denorm_min();
    EXPECT_TRUE(math::Distance::fromMeters(2.0 * smallest).equals(
        math::Distance::fromMeters(smallest), 0.5));
}
