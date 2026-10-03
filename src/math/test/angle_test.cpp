#include "math/angle.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <type_traits>

static_assert(!std::is_default_constructible<math::Angle>::value,
    "Angles must be created through a unit factory");
static_assert(!std::is_constructible<math::Angle, double>::value,
    "Angles must be created through a unit factory");

TEST(AngleTest, CreatesAnglesFromDegreeLiterals)
{
    using namespace math::literals;

    static_assert(std::is_same<decltype(30_deg), math::Angle>::value,
        "Degree literals must produce an Angle");
    EXPECT_DOUBLE_EQ(30_deg.deg(), 30.0);
    EXPECT_DOUBLE_EQ(30.5_deg.deg(), 30.5);
    EXPECT_DOUBLE_EQ((-30_deg).deg(), -30.0);
    EXPECT_DOUBLE_EQ(0_deg.deg(), 0.0);
    EXPECT_NEAR(180_deg.rad(), math::PI, 1e-15);
    EXPECT_DOUBLE_EQ((30_deg + 15.5_deg).deg(), 45.5);
}

TEST(AngleTest, WrapsToSignedDegreeRange)
{
    using namespace math::literals;

    EXPECT_EQ(0_deg.wrap180(), 0_deg);
    EXPECT_EQ(179.5_deg.wrap180(), 179.5_deg);
    EXPECT_EQ((-179.5_deg).wrap180(), -179.5_deg);
    EXPECT_EQ(180_deg.wrap180(), -180_deg);
    EXPECT_EQ((-180_deg).wrap180(), -180_deg);
    EXPECT_EQ(181_deg.wrap180(), -179_deg);
    EXPECT_EQ((-181_deg).wrap180(), 179_deg);
    EXPECT_EQ(540_deg.wrap180(), -180_deg);
    EXPECT_EQ((-540_deg).wrap180(), -180_deg);
    EXPECT_EQ(1080_deg.wrap180(), 0_deg);
    EXPECT_EQ((-1080_deg).wrap180(), 0_deg);
}

TEST(AngleTest, WrappingDoesNotChangeOriginalAngle)
{
    using namespace math::literals;

    const auto angle = 450_deg;
    EXPECT_EQ(angle.wrap180(), 90_deg);
    EXPECT_EQ(angle, 450_deg);
}

TEST(AngleTest, ConvertsRepresentativeDegreesToRadians)
{
    EXPECT_DOUBLE_EQ(math::Angle::fromDegrees(0.0).rad(), 0.0);
    EXPECT_NEAR(math::Angle::fromDegrees(45.0).rad(), math::PI / 4.0, 1e-15);
    EXPECT_NEAR(math::Angle::fromDegrees(90.0).rad(), math::PI / 2.0, 1e-15);
    EXPECT_NEAR(math::Angle::fromDegrees(180.0).rad(), math::PI, 1e-15);
    EXPECT_NEAR(math::Angle::fromDegrees(-90.0).rad(), -math::PI / 2.0, 1e-15);
    EXPECT_NEAR(math::Angle::fromDegrees(-180.0).rad(), -math::PI, 1e-15);
}

TEST(AngleTest, ConvertsRepresentativeRadiansToDegrees)
{
    EXPECT_DOUBLE_EQ(math::Angle::fromRadians(0.0).deg(), 0.0);
    EXPECT_NEAR(math::Angle::fromRadians(math::PI / 4.0).deg(), 45.0, 1e-12);
    EXPECT_NEAR(math::Angle::fromRadians(math::PI / 2.0).deg(), 90.0, 1e-12);
    EXPECT_NEAR(math::Angle::fromRadians(math::PI).deg(), 180.0, 1e-12);
    EXPECT_NEAR(math::Angle::fromRadians(-math::PI / 2.0).deg(), -90.0, 1e-12);
    EXPECT_NEAR(math::Angle::fromRadians(-math::PI).deg(), -180.0, 1e-12);
}

TEST(AngleTest, ConvertsBetweenUnitsInBothDirections)
{
    for (const double degrees : {-360.0, -123.4, -1.0, 0.0, 1.0, 123.4, 360.0})
    {
        EXPECT_NEAR(math::Angle::fromRadians(math::Angle::fromDegrees(degrees).rad()).deg(), degrees, 1e-12);
    }

    for (const double radians : {-2.5, -math::PI, -0.1, 0.0, 0.1, math::PI, 2.5})
    {
        EXPECT_NEAR(math::Angle::fromDegrees(math::Angle::fromRadians(radians).deg()).rad(), radians, 1e-12);
    }
}

TEST(AngleTest, StoresDegreesWithoutWrappingAndExportsBothUnits)
{
    EXPECT_DOUBLE_EQ(math::Angle::fromDegrees(0.0).deg(), 0.0);
    EXPECT_DOUBLE_EQ(math::Angle::fromDegrees(0.0).rad(), 0.0);

    for (const double degrees : {-720.0, -123.4, 0.0, 90.0, 450.0})
    {
        const auto angle = math::Angle::fromDegrees(degrees);
        EXPECT_DOUBLE_EQ(angle.deg(), degrees);
        EXPECT_NEAR(angle.rad(), degrees * (math::PI / 180.0), 1e-15);
        EXPECT_DOUBLE_EQ(math::Angle::fromDegrees(degrees).deg(), degrees);
        EXPECT_NEAR(math::Angle::fromRadians(angle.rad()).deg(), degrees, 1e-12);
    }
}

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

TEST(AngleTest, RejectsDivisionByZeroWithoutChangingValue)
{
    auto angle = math::Angle::fromDegrees(45.0);
    EXPECT_THROW(angle / 0.0, std::domain_error);
    EXPECT_THROW(angle /= -0.0, std::domain_error);
    EXPECT_DOUBLE_EQ(angle.deg(), 45.0);
}

TEST(AngleTest, ComparesStoredDegreesWithoutWrapping)
{
    const auto zero = math::Angle::fromDegrees(0.0);
    const auto negative = math::Angle::fromDegrees(-10.0);
    const auto fullTurn = math::Angle::fromDegrees(360.0);

    EXPECT_TRUE(zero == math::Angle::fromDegrees(0.0));
    EXPECT_FALSE(zero != math::Angle::fromDegrees(0.0));
    EXPECT_TRUE(zero != fullTurn);
    EXPECT_FALSE(zero == fullTurn);
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
