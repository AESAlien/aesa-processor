#include "math/distance.hpp"

#include <gtest/gtest.h>

#include <type_traits>

static_assert(!std::is_default_constructible<math::Distance>::value,
    "Distances must be created through a unit factory");
static_assert(!std::is_constructible<math::Distance, double>::value,
    "Distances must be created through a unit factory");

TEST(DistanceTest, StoresKilometersWithoutChangingValue)
{
    const double values[] = {-1234.567, -0.001, 0.0, 0.001, 1234.567, 1000000.0};

    for (const double value : values)
    {
        const auto distance = math::Distance::fromKilometers(value);
        EXPECT_DOUBLE_EQ(distance.km(), value);
    }
}

TEST(DistanceTest, CreatesDistancesFromKilometerLiterals)
{
    using namespace math::literals;

    static_assert(std::is_same<decltype(2_km), math::Distance>::value,
        "Integer kilometer literals must produce a Distance");
    static_assert(std::is_same<decltype(2.5_km), math::Distance>::value,
        "Floating-point kilometer literals must produce a Distance");
    EXPECT_DOUBLE_EQ(2_km.km(), 2.0);
    EXPECT_DOUBLE_EQ(2.5_km.km(), 2.5);
    EXPECT_DOUBLE_EQ(0_km.km(), 0.0);
    EXPECT_DOUBLE_EQ(0.001_km.km(), 0.001);
    EXPECT_DOUBLE_EQ((-0.5_km).km(), -0.5);
}

TEST(DistanceTest, ComparesStoredKilometers)
{
    using namespace math::literals;

    const auto zero = 0_km;
    const auto negative = -0.5_km;
    const auto positive = 2_km;

    EXPECT_TRUE(zero == 0.0_km);
    EXPECT_FALSE(zero != 0.0_km);
    EXPECT_TRUE(zero != positive);
    EXPECT_FALSE(zero == positive);
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
