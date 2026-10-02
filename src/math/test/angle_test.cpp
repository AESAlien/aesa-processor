#include "math/angle.hpp"

#include <gtest/gtest.h>

TEST(AngleTest, ConvertsRepresentativeDegreesToRadians)
{
    EXPECT_DOUBLE_EQ(math::degToRad(0.0), 0.0);
    EXPECT_NEAR(math::degToRad(45.0), math::PI / 4.0, 1e-15);
    EXPECT_NEAR(math::degToRad(90.0), math::PI / 2.0, 1e-15);
    EXPECT_NEAR(math::degToRad(180.0), math::PI, 1e-15);
    EXPECT_NEAR(math::degToRad(-90.0), -math::PI / 2.0, 1e-15);
    EXPECT_NEAR(math::degToRad(-180.0), -math::PI, 1e-15);
}

TEST(AngleTest, ConvertsRepresentativeRadiansToDegrees)
{
    EXPECT_DOUBLE_EQ(math::radToDeg(0.0), 0.0);
    EXPECT_NEAR(math::radToDeg(math::PI / 4.0), 45.0, 1e-12);
    EXPECT_NEAR(math::radToDeg(math::PI / 2.0), 90.0, 1e-12);
    EXPECT_NEAR(math::radToDeg(math::PI), 180.0, 1e-12);
    EXPECT_NEAR(math::radToDeg(-math::PI / 2.0), -90.0, 1e-12);
    EXPECT_NEAR(math::radToDeg(-math::PI), -180.0, 1e-12);
}

TEST(AngleTest, ConvertsBetweenUnitsInBothDirections)
{
    for (const double degrees : {-360.0, -123.4, -1.0, 0.0, 1.0, 123.4, 360.0})
    {
        EXPECT_NEAR(math::radToDeg(math::degToRad(degrees)), degrees, 1e-12);
    }

    for (const double radians : {-2.5, -math::PI, -0.1, 0.0, 0.1, math::PI, 2.5})
    {
        EXPECT_NEAR(math::degToRad(math::radToDeg(radians)), radians, 1e-12);
    }
}
