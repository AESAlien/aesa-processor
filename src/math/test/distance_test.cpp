#include "math/geometry/distance.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <type_traits>
#include <utility>

using namespace math::literals;

static_assert(!std::is_default_constructible_v<math::Distance>);
static_assert(!std::is_constructible_v<math::Distance, double>);
static_assert(std::is_same_v<
    decltype(std::declval<math::Distance>() + std::declval<math::Length>()),
    math::Distance>);
static_assert(std::is_same_v<
    decltype(std::declval<math::Length>() + std::declval<math::Distance>()),
    math::Distance>);
static_assert(std::is_same_v<
    decltype(std::declval<math::Distance>() / std::declval<math::Distance>()),
    double>);

TEST(DistanceTest, CreatesFromLengthAndUnits)
{
    const math::Distance distance(20_km);

    EXPECT_DOUBLE_EQ(distance.km(), 20.0);
    EXPECT_DOUBLE_EQ(distance.m(), 20000.0);
    EXPECT_EQ(math::Distance::fromMeters(3000.0), 3_km);
    EXPECT_EQ(math::Distance::fromKilometers(4.0), 4_km);
    EXPECT_EQ(math::Distance(0_m), 0_m);
}

TEST(DistanceTest, SupportsLengthArithmetic)
{
    math::Distance distance(5_km);

    EXPECT_EQ(distance + 2_km, 7_km);
    EXPECT_EQ(2_km + distance, 7_km);
    EXPECT_EQ(distance - 2_km, 3_km);

    distance += 2_km;
    EXPECT_EQ(distance, 7_km);
    distance -= 3_km;
    EXPECT_EQ(distance, 4_km);
    EXPECT_TRUE(distance > 3_km);
    EXPECT_TRUE(3_km < distance);
}

TEST(DistanceTest, RejectsNegativeDistances)
{
    EXPECT_THROW(math::Distance(-1_m), std::invalid_argument);
    EXPECT_THROW(math::Distance::fromMeters(-1.0), std::invalid_argument);
    EXPECT_THROW(math::Distance::fromKilometers(-1.0), std::invalid_argument);
    EXPECT_THROW(math::Distance(1_m) - 2_m, std::invalid_argument);
}

TEST(DistanceTest, ComparesWithDistancesAndLengths)
{
    const math::Distance distance(2_km);
    const math::Distance slightlyLarger(2000.001_m);
    const math::QuantityTolerance<math::Length> noTolerance{0_m, 0.0};

    EXPECT_TRUE(distance.equals(math::Distance(2000_m)));
    EXPECT_TRUE(distance.equals(2000_m));
    EXPECT_FALSE(distance.notEquals(2000_m));
    EXPECT_TRUE(distance.notEquals(slightlyLarger, noTolerance));
    EXPECT_TRUE(distance.lessThan(slightlyLarger, noTolerance));
    EXPECT_TRUE(distance.lessThan(2000.001_m, noTolerance));
    EXPECT_TRUE(distance.lessThanOrEqual(2000_m, noTolerance));
    EXPECT_TRUE(distance.greaterThan(1999.999_m, noTolerance));
    EXPECT_TRUE(distance.greaterThanOrEqual(math::Distance(2000_m), noTolerance));
    EXPECT_EQ(distance, 2000_m);
    EXPECT_EQ(2000_m, distance);
    EXPECT_NE(distance, 3_km);
    EXPECT_NE(3_km, distance);
    EXPECT_TRUE(distance < 2000.001_m);
    EXPECT_TRUE(1999.999_m < distance);
}

TEST(DistanceTest, DividesDistancesToProduceUnitlessRatio)
{
    EXPECT_DOUBLE_EQ(math::Distance(6_m) / math::Distance(3_m), 2.0);
    EXPECT_THROW(math::Distance(1_m) / math::Distance(0_m), std::domain_error);
}
