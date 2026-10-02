#include "math/angle.hpp"

#include <gtest/gtest.h>

TEST(AngleTest, ConvertsRepresentativeDegreesToRadians)
{
    EXPECT_DOUBLE_EQ(math::DegToRad(0.0), 0.0);
    EXPECT_NEAR(math::DegToRad(45.0), math::Pi / 4.0, 1e-15);
    EXPECT_NEAR(math::DegToRad(90.0), math::Pi / 2.0, 1e-15);
    EXPECT_NEAR(math::DegToRad(180.0), math::Pi, 1e-15);
    EXPECT_NEAR(math::DegToRad(-90.0), -math::Pi / 2.0, 1e-15);
    EXPECT_NEAR(math::DegToRad(-180.0), -math::Pi, 1e-15);
}

TEST(AngleTest, ConvertsRepresentativeRadiansToDegrees)
{
    EXPECT_DOUBLE_EQ(math::RadToDeg(0.0), 0.0);
    EXPECT_NEAR(math::RadToDeg(math::Pi / 4.0), 45.0, 1e-12);
    EXPECT_NEAR(math::RadToDeg(math::Pi / 2.0), 90.0, 1e-12);
    EXPECT_NEAR(math::RadToDeg(math::Pi), 180.0, 1e-12);
    EXPECT_NEAR(math::RadToDeg(-math::Pi / 2.0), -90.0, 1e-12);
    EXPECT_NEAR(math::RadToDeg(-math::Pi), -180.0, 1e-12);
}

TEST(AngleTest, ConvertsBetweenUnitsInBothDirections)
{
    for (const double degrees : {-360.0, -123.4, -1.0, 0.0, 1.0, 123.4, 360.0})
    {
        EXPECT_NEAR(math::RadToDeg(math::DegToRad(degrees)), degrees, 1e-12);
    }

    for (const double radians : {-2.5, -math::Pi, -0.1, 0.0, 0.1, math::Pi, 2.5})
    {
        EXPECT_NEAR(math::DegToRad(math::RadToDeg(radians)), radians, 1e-12);
    }
}
