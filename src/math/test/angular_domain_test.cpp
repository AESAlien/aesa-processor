#include <math/geometry/angular_width.hpp>
#include <math/geometry/azimuth.hpp>
#include <math/geometry/elevation.hpp>

#include <gtest/gtest.h>

#include <stdexcept>

namespace
{
using namespace math::literals;

TEST(AzimuthTest, NormalizesAngleToSingleTurn)
{
    EXPECT_EQ(math::Azimuth(-30_deg), 330_deg);
    EXPECT_EQ(30_deg, math::Azimuth(390_deg));
}

TEST(AzimuthTest, CreatesDirectlyFromDegreesAndRadians)
{
    EXPECT_EQ(math::Azimuth::fromDegrees(-30.0), 330_deg);
    EXPECT_EQ(math::Azimuth::fromRadians(math::PI / 6.0), 30_deg);
}

TEST(AzimuthTest, TreatsFullTurnApartValuesAsEquivalent)
{
    EXPECT_EQ(math::Azimuth(0_deg), math::Azimuth(360_deg));
}

TEST(AzimuthTest, SupportsAngleArithmeticAndNormalizesResults)
{
    math::Azimuth azimuth(350_deg);

    EXPECT_EQ(azimuth + 20_deg, 10_deg);
    EXPECT_EQ(20_deg + azimuth, 10_deg);
    EXPECT_EQ(azimuth - 20_deg, 330_deg);

    azimuth += 20_deg;
    EXPECT_EQ(azimuth, 10_deg);
    azimuth -= 40_deg;
    EXPECT_EQ(azimuth, 330_deg);
}

TEST(ElevationTest, AcceptsClosedPhysicalRange)
{
    EXPECT_EQ(math::Elevation(-90_deg), -90_deg);
    EXPECT_EQ(math::Elevation(90_deg), 90_deg);
}

TEST(ElevationTest, CreatesDirectlyFromDegreesAndRadians)
{
    EXPECT_EQ(math::Elevation::fromDegrees(-30.0), -30_deg);
    EXPECT_EQ(math::Elevation::fromRadians(math::PI / 6.0), 30_deg);
    EXPECT_THROW(math::Elevation::fromRadians(math::PI), std::invalid_argument);
}

TEST(ElevationTest, RejectsAnglesOutsidePhysicalRange)
{
    EXPECT_THROW(math::Elevation(-90.1_deg), std::invalid_argument);
    EXPECT_THROW(math::Elevation(90.1_deg), std::invalid_argument);
}

TEST(ElevationTest, SupportsAngleArithmeticAndPreservesRange)
{
    math::Elevation elevation(10_deg);

    EXPECT_EQ(elevation + 20_deg, 30_deg);
    EXPECT_EQ(20_deg + elevation, 30_deg);
    EXPECT_EQ(elevation - 20_deg, -10_deg);

    elevation += 15_deg;
    EXPECT_EQ(elevation, 25_deg);
    elevation -= 30_deg;
    EXPECT_EQ(-5_deg, elevation);

    EXPECT_THROW(elevation + 100_deg, std::invalid_argument);
    EXPECT_THROW(elevation - 100_deg, std::invalid_argument);
}

TEST(AngularWidthTest, AcceptsPositiveAngle)
{
    EXPECT_EQ(math::AngularWidth(3_deg), 3_deg);
}

TEST(AngularWidthTest, RejectsZeroAndNegativeAngles)
{
    EXPECT_THROW(math::AngularWidth(0_deg), std::invalid_argument);
    EXPECT_THROW(math::AngularWidth(-1_deg), std::invalid_argument);
}

TEST(AngularWidthTest, SupportsAngleArithmeticAndPreservesPositivity)
{
    math::AngularWidth width(3_deg);

    EXPECT_EQ(width + 2_deg, 5_deg);
    EXPECT_EQ(2_deg + width, 5_deg);
    EXPECT_EQ(width - 1_deg, 2_deg);

    width += 2_deg;
    EXPECT_EQ(width, 5_deg);
    width -= 1_deg;
    EXPECT_EQ(4_deg, width);

    EXPECT_THROW(width - 4_deg, std::invalid_argument);
    EXPECT_THROW(width - 5_deg, std::invalid_argument);
}

}
