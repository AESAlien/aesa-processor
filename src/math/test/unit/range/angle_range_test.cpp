#include "math/range/angle_range.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

using namespace math::literals;

TEST(AngleRangeTest, AcceptsTiltAngleBoundsAndValues)
{
    const math::AngleRange range(math::TiltAngle(-10_deg), math::TiltAngle(60_deg));

    EXPECT_DOUBLE_EQ(range.start().deg(), -10.0);
    EXPECT_DOUBLE_EQ(range.end().deg(), 60.0);
    EXPECT_TRUE(range.contains(math::TiltAngle(-10_deg)));
    EXPECT_TRUE(range.contains(math::TiltAngle(0_deg)));
    EXPECT_TRUE(range.contains(math::TiltAngle(60_deg)));
    EXPECT_FALSE(range.contains(math::TiltAngle(-11_deg)));
    EXPECT_FALSE(range.contains(math::TiltAngle(61_deg)));
    EXPECT_THROW(math::AngleRange(math::TiltAngle(10_deg), math::TiltAngle(10_deg)), std::invalid_argument);
    EXPECT_THROW(math::AngleRange(math::TiltAngle(20_deg), math::TiltAngle(10_deg)), std::invalid_argument);
}

TEST(AngleRangeTest, TiltAngleContainsUsesTheSameToleranceAsAngleContains)
{
    const math::AngleRange range(-10_deg, 60_deg);
    const math::TiltAngle value(-10.000000001_deg);

    EXPECT_TRUE(range.contains(value));
    EXPECT_FALSE(range.contains(value, math::Angle::NONE_TOLERANCE));
    const math::QuantityTolerance<math::Angle> tolerance{0.25_deg, 0.0};
    EXPECT_TRUE(range.contains(math::TiltAngle(-10.25_deg), tolerance));
    EXPECT_TRUE(range.contains(math::TiltAngle(60.25_deg), tolerance));
    EXPECT_FALSE(range.contains(math::TiltAngle(-10.3_deg), tolerance));
    EXPECT_FALSE(range.contains(math::TiltAngle(60.3_deg), tolerance));
}

TEST(AngleRangeTest, PreservesBoundsAndIncludesClosedInterval)
{
    const math::AngleRange range(-60_deg, 60_deg);

    EXPECT_DOUBLE_EQ(range.start().deg(), -60.0);
    EXPECT_DOUBLE_EQ(range.end().deg(), 60.0);
    EXPECT_TRUE(range.contains(-60_deg));
    EXPECT_TRUE(range.contains(0_deg));
    EXPECT_TRUE(range.contains(60_deg));
    EXPECT_FALSE(range.contains(-60.1_deg));
    EXPECT_FALSE(range.contains(60.1_deg));
}

TEST(AngleRangeTest, RejectsEqualAndReversedBounds)
{
    EXPECT_THROW(math::AngleRange(10_deg, 10_deg), std::invalid_argument);
    EXPECT_THROW(math::AngleRange(20_deg, 10_deg), std::invalid_argument);
    EXPECT_THROW(math::AngleRange(350_deg, 10_deg), std::invalid_argument);
}

TEST(AngleRangeTest, ValidatesOrderingWithoutTolerance)
{
    EXPECT_NO_THROW(math::AngleRange(0_deg, 0.0000000000001_deg));
    EXPECT_THROW(math::AngleRange(0.0000000000001_deg, 0_deg), std::invalid_argument);
}

TEST(AngleRangeTest, AppliesDefaultOrExplicitToleranceAtBothBounds)
{
    const math::AngleRange range(10_deg, 20_deg);

    EXPECT_TRUE(range.contains(9.999999999_deg));
    EXPECT_TRUE(range.contains(20.000000001_deg));
    EXPECT_FALSE(range.contains(9.999999999_deg, math::Angle::NONE_TOLERANCE));
    EXPECT_FALSE(range.contains(20.000000001_deg, math::Angle::NONE_TOLERANCE));

    const math::QuantityTolerance<math::Angle> tolerance{0.25_deg, 0.0};
    EXPECT_TRUE(range.contains(9.75_deg, tolerance));
    EXPECT_TRUE(range.contains(20.25_deg, tolerance));
    EXPECT_FALSE(range.contains(9.7_deg, tolerance));
    EXPECT_FALSE(range.contains(20.3_deg, tolerance));
}

TEST(AngleRangeTest, DoesNotNormalizeBoundsOrValues)
{
    const math::AngleRange range(350_deg, 370_deg);

    EXPECT_TRUE(range.contains(360_deg));
    EXPECT_FALSE(range.contains(0_deg));
    EXPECT_DOUBLE_EQ(range.end().deg(), 370.0);
    EXPECT_TRUE(math::AngleRange(-180_deg, 180_deg).contains(180_deg));
    EXPECT_TRUE(math::AngleRange(-180_deg, 180_deg).contains(-180_deg));
}

TEST(AngleRangeTest, AcceptsRadiansAndRejectsNonfiniteInputThroughAngle)
{
    const math::AngleRange range(0_rad, math::Angle::fromRadians(math::PI));
    EXPECT_TRUE(range.contains(math::Angle::fromRadians(math::PI / 2.0)));

    EXPECT_THROW(
        math::AngleRange(0_deg, math::Angle::fromDegrees(std::numeric_limits<double>::infinity())),
        std::invalid_argument);
    EXPECT_THROW(
        math::AngleRange(math::Angle::fromDegrees(std::numeric_limits<double>::quiet_NaN()), 1_deg),
        std::invalid_argument);
}
