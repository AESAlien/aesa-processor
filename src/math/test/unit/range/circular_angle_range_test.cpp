#include "math/range/circular_angle_range.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

using namespace math::literals;

TEST(CircularAngleRangeTest, ContainsClosedArcAcrossZero)
{
    const math::CircularAngleRange range(-60_deg, 60_deg);
    EXPECT_EQ(range.start(), 300_deg);
    EXPECT_EQ(range.end(), 60_deg);
    EXPECT_TRUE(range.contains(0_deg));
    EXPECT_TRUE(range.contains(300_deg));
    EXPECT_TRUE(range.contains(60_deg));
    EXPECT_FALSE(range.contains(299_deg));
    EXPECT_FALSE(range.contains(61_deg));
    EXPECT_FALSE(range.contains(180_deg));
}

TEST(CircularAngleRangeTest, ContainsIncreasingArcRatherThanShortestArc)
{
    const math::CircularAngleRange range(60_deg, -60_deg);
    EXPECT_TRUE(range.contains(180_deg));
    EXPECT_FALSE(range.contains(0_deg));
    const math::CircularAngleRange narrow(350_deg, 10_deg);
    EXPECT_TRUE(narrow.contains(0_deg));
    EXPECT_FALSE(narrow.contains(180_deg));
}

TEST(CircularAngleRangeTest, IncludesEquivalentRearBoundaries)
{
    const math::CircularAngleRange range(170_deg, 180_deg);
    EXPECT_TRUE(range.contains(-180_deg));
    EXPECT_TRUE(range.contains(180_deg));
    EXPECT_FALSE(range.contains(181_deg));
    EXPECT_TRUE(math::CircularAngleRange(-180_deg, -170_deg).contains(180_deg));
}

TEST(CircularAngleRangeTest, DistinguishesFullCircleFromEqualBounds)
{
    for (const auto& range : {math::CircularAngleRange::fullCircle(),
        math::CircularAngleRange(-180_deg, 180_deg), math::CircularAngleRange(360_deg, 0_deg)})
    {
        EXPECT_TRUE(range.isFullCircle());
        EXPECT_TRUE(range.contains(0_deg));
        EXPECT_TRUE(range.contains(180_deg));
        EXPECT_TRUE(range.contains(359_deg));
    }
    EXPECT_FALSE(math::CircularAngleRange(0_deg, 359.999_deg).isFullCircle());
    EXPECT_THROW(math::CircularAngleRange(10_deg, 10_deg), std::invalid_argument);
    EXPECT_THROW(math::CircularAngleRange(math::CircularAngle(0_deg), math::CircularAngle(360_deg)), std::invalid_argument);
}

TEST(CircularAngleRangeTest, AcceptsCircularAnglesAndNormalizesInput)
{
    const math::CircularAngleRange range(math::CircularAngle(350_deg), math::CircularAngle(10_deg));
    EXPECT_TRUE(range.contains(math::CircularAngle(0_deg)));
    EXPECT_TRUE(range.contains(720_deg));
    EXPECT_TRUE(range.contains(-360_deg));
    EXPECT_FALSE(range.contains(math::CircularAngle(180_deg)));
    const math::CircularAngleRange wide(math::CircularAngle(60_deg), math::CircularAngle(-60_deg));
    EXPECT_TRUE(wide.contains(math::CircularAngle(180_deg)));
    EXPECT_FALSE(wide.contains(math::CircularAngle(0_deg)));
    EXPECT_DOUBLE_EQ(wide.start().angle().deg(), 60.0);
    EXPECT_DOUBLE_EQ(wide.end().angle().deg(), 300.0);
}

TEST(CircularAngleRangeTest, AppliesToleranceAcrossSeamAtBothBoundaries)
{
    const math::CircularAngleRange range(0_deg, 10_deg);
    const math::QuantityTolerance<math::Angle> tolerance{0.25_deg, 0.0};
    EXPECT_TRUE(range.contains(359.75_deg, tolerance));
    EXPECT_TRUE(range.contains(10.25_deg, tolerance));
    EXPECT_FALSE(range.contains(359.7_deg, tolerance));
    EXPECT_FALSE(range.contains(10.3_deg, tolerance));
    EXPECT_FALSE(range.contains(359.75_deg, math::Angle::NONE_TOLERANCE));
    EXPECT_FALSE(range.contains(10.25_deg, math::Angle::NONE_TOLERANCE));
    EXPECT_TRUE(range.contains(10.0000000000001_deg));
}

TEST(CircularAngleRangeTest, RejectsMoreThanOneTurnAndNonfiniteInput)
{
    EXPECT_THROW(math::CircularAngleRange(0_deg, 360.1_deg), std::invalid_argument);
    EXPECT_THROW(math::CircularAngleRange(360.1_deg, 0_deg), std::invalid_argument);
    const auto largest = math::Angle::fromDegrees(std::numeric_limits<double>::max());
    EXPECT_THROW(math::CircularAngleRange(-largest, largest), std::invalid_argument);
    EXPECT_THROW(math::CircularAngleRange(0_deg, math::Angle::fromDegrees(
        std::numeric_limits<double>::infinity())), std::invalid_argument);
}
