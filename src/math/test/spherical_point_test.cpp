#include "math/spherical_point.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

using namespace math::literals;

TEST(SphericalPointTest, StoresValidCoordinates)
{
    const math::SphericalPoint point(2_km, -180_deg, 90_deg);

    EXPECT_DOUBLE_EQ(point.radius().km(), 2.0);
    EXPECT_DOUBLE_EQ(point.azimuth().deg(), -180.0);
    EXPECT_DOUBLE_EQ(point.elevation().deg(), 90.0);

    const math::SphericalPoint origin(0_m, 180_deg, -90_deg);
    EXPECT_DOUBLE_EQ(origin.radius().m(), 0.0);
}

TEST(SphericalPointTest, RejectsInvalidRadius)
{
    EXPECT_THROW((math::SphericalPoint(-1_m, 0_deg, 0_deg)), std::invalid_argument);
    EXPECT_THROW((math::SphericalPoint(
        math::Distance::fromMeters(std::numeric_limits<double>::infinity()),
        0_deg,
        0_deg)), std::invalid_argument);
    EXPECT_THROW((math::SphericalPoint(
        math::Distance::fromMeters(std::numeric_limits<double>::quiet_NaN()),
        0_deg,
        0_deg)), std::invalid_argument);
}

TEST(SphericalPointTest, RejectsAzimuthOutsideInclusiveRange)
{
    EXPECT_THROW((math::SphericalPoint(1_m, -180.001_deg, 0_deg)), std::invalid_argument);
    EXPECT_THROW((math::SphericalPoint(1_m, 180.001_deg, 0_deg)), std::invalid_argument);
    EXPECT_THROW((math::SphericalPoint(
        1_m,
        math::Angle::fromDegrees(std::numeric_limits<double>::infinity()),
        0_deg)), std::invalid_argument);
}

TEST(SphericalPointTest, RejectsElevationOutsideInclusiveRange)
{
    EXPECT_THROW((math::SphericalPoint(1_m, 0_deg, -90.001_deg)), std::invalid_argument);
    EXPECT_THROW((math::SphericalPoint(1_m, 0_deg, 90.001_deg)), std::invalid_argument);
    EXPECT_THROW((math::SphericalPoint(
        1_m,
        0_deg,
        math::Angle::fromDegrees(std::numeric_limits<double>::quiet_NaN()))), std::invalid_argument);
}
