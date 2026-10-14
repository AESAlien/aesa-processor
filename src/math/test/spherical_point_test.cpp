#include "math/geometry/spherical_point.hpp"
#include "math/trigonometry.hpp"
#include "math/linear_algebra/vector.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

using namespace math::literals;

TEST(SphericalPointTest, StoresValidCoordinates)
{
    const math::SphericalDirection direction(-180_deg, 90_deg);
    const math::SphericalPoint point(2_km, direction);

    EXPECT_DOUBLE_EQ(point.radius().km(), 2.0);
    EXPECT_DOUBLE_EQ(point.direction().azimuth().deg(), -180.0);
    EXPECT_DOUBLE_EQ(point.direction().elevation().deg(), 90.0);

    const math::SphericalPoint origin(0_m, math::SphericalDirection(180_deg, -90_deg));
    EXPECT_DOUBLE_EQ(origin.radius().m(), 0.0);
}

TEST(SphericalPointTest, RejectsInvalidRadius)
{
    EXPECT_THROW((math::SphericalPoint(-1_m, math::SphericalDirection(0_deg, 0_deg))), std::invalid_argument);
    EXPECT_THROW((math::SphericalPoint(
        math::Distance::fromMeters(std::numeric_limits<double>::infinity()),
        math::SphericalDirection(0_deg, 0_deg))), std::invalid_argument);
    EXPECT_THROW((math::SphericalPoint(
        math::Distance::fromMeters(std::numeric_limits<double>::quiet_NaN()),
        math::SphericalDirection(0_deg, 0_deg))), std::invalid_argument);
}

TEST(SphericalDirectionTest, RejectsAzimuthOutsideInclusiveRange)
{
    EXPECT_THROW((math::SphericalDirection(-180.001_deg, 0_deg)), std::invalid_argument);
    EXPECT_THROW((math::SphericalDirection(180.001_deg, 0_deg)), std::invalid_argument);
    EXPECT_THROW((math::SphericalDirection(
        math::Angle::fromDegrees(std::numeric_limits<double>::infinity()),
        0_deg)), std::invalid_argument);
}

TEST(SphericalDirectionTest, RejectsElevationOutsideInclusiveRange)
{
    EXPECT_THROW((math::SphericalDirection(0_deg, -90.001_deg)), std::invalid_argument);
    EXPECT_THROW((math::SphericalDirection(0_deg, 90.001_deg)), std::invalid_argument);
    EXPECT_THROW((math::SphericalDirection(
        0_deg,
        math::Angle::fromDegrees(std::numeric_limits<double>::quiet_NaN()))), std::invalid_argument);
}

TEST(SphericalDirectionTest, ConvertsToAndFromCartesianUnitVectors)
{
    const math::SphericalDirection direction(30_deg, 40_deg);
    const math::Vector vector = direction.toCartesianUnitVector();

    EXPECT_NEAR(vector(0), math::cos(40_deg) * math::sin(30_deg), 1e-12);
    EXPECT_NEAR(vector(1), math::cos(40_deg) * math::cos(30_deg), 1e-12);
    EXPECT_NEAR(vector(2), math::sin(40_deg), 1e-12);
    EXPECT_NEAR(vector.norm(), 1.0, 1e-12);

    const math::SphericalDirection restored = math::SphericalDirection::fromCartesianUnitVector(vector * 7.0);
    EXPECT_NEAR(restored.azimuth().deg(), 30.0, 1e-12);
    EXPECT_NEAR(restored.elevation().deg(), 40.0, 1e-12);
}

TEST(SphericalDirectionTest, ConvertsVerticalCartesianUnitVectors)
{
    const math::SphericalDirection up = math::SphericalDirection::fromCartesianUnitVector(
        math::Vector{0.0, 0.0, 2.0});
    EXPECT_DOUBLE_EQ(up.azimuth().deg(), 0.0);
    EXPECT_DOUBLE_EQ(up.elevation().deg(), 90.0);
}

TEST(SphericalDirectionTest, ConvertsLargeFiniteCartesianVectorsWithoutOverflow)
{
    const double max = std::numeric_limits<double>::max();
    const math::SphericalDirection direction =
        math::SphericalDirection::fromCartesianUnitVector(math::Vector{max, max, max});

    EXPECT_DOUBLE_EQ(direction.azimuth().deg(), 45.0);
    EXPECT_NEAR(direction.elevation().deg(), 35.264389682754654, 1e-12);
}

TEST(SphericalDirectionTest, RejectsInvalidCartesianVectors)
{
    EXPECT_THROW((math::SphericalDirection::fromCartesianUnitVector(math::Vector{1.0, 2.0})), std::invalid_argument);
    EXPECT_THROW((math::SphericalDirection::fromCartesianUnitVector(math::Vector{0.0, 0.0, 0.0})),
        std::invalid_argument);
    EXPECT_THROW((math::SphericalDirection::fromCartesianUnitVector(math::Vector{
        0.0,
        0.0,
        std::numeric_limits<double>::infinity()})), std::invalid_argument);
}
