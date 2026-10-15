#include "math/geometry/cartesian_position.hpp"
#include "math/geometry/cartesian_velocity.hpp"
#include "math/geometry/spherical_position.hpp"
#include "math/trigonometry.hpp"
#include "math/linear_algebra/vector.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <type_traits>

using namespace math::literals;

static_assert(std::is_const_v<decltype(math::CartesianPosition::x)>);
static_assert(std::is_const_v<decltype(math::CartesianPosition::y)>);
static_assert(std::is_const_v<decltype(math::CartesianPosition::z)>);
static_assert(std::is_same_v<
    std::remove_const_t<decltype(math::CartesianPosition::x)>,
    math::Length>);
static_assert(std::is_same_v<
    std::remove_const_t<decltype(math::CartesianPosition::y)>,
    math::Length>);
static_assert(std::is_same_v<
    std::remove_const_t<decltype(math::CartesianPosition::z)>,
    math::Length>);
static_assert(std::is_copy_constructible_v<math::CartesianPosition>);
static_assert(!std::is_copy_assignable_v<math::CartesianPosition>);
static_assert(!std::is_move_assignable_v<math::CartesianPosition>);
static_assert(!std::is_aggregate_v<math::CartesianPosition>);
static_assert(std::is_const_v<decltype(math::CartesianVelocity::x)>);
static_assert(std::is_const_v<decltype(math::CartesianVelocity::y)>);
static_assert(std::is_const_v<decltype(math::CartesianVelocity::z)>);
static_assert(std::is_copy_constructible_v<math::CartesianVelocity>);
static_assert(!std::is_copy_assignable_v<math::CartesianVelocity>);
static_assert(!std::is_move_assignable_v<math::CartesianVelocity>);
static_assert(!std::is_aggregate_v<math::CartesianVelocity>);
static_assert(std::is_const_v<decltype(math::SphericalDirection::azimuth)>);
static_assert(std::is_const_v<decltype(math::SphericalDirection::elevation)>);
static_assert(std::is_copy_constructible_v<math::SphericalDirection>);
static_assert(!std::is_copy_assignable_v<math::SphericalDirection>);
static_assert(!std::is_move_assignable_v<math::SphericalDirection>);
static_assert(!std::is_aggregate_v<math::SphericalDirection>);
static_assert(std::is_const_v<decltype(math::SphericalPosition::radius)>);
static_assert(std::is_const_v<decltype(math::SphericalPosition::direction)>);
static_assert(std::is_copy_constructible_v<math::SphericalPosition>);
static_assert(!std::is_copy_assignable_v<math::SphericalPosition>);
static_assert(!std::is_move_assignable_v<math::SphericalPosition>);
static_assert(!std::is_aggregate_v<math::SphericalPosition>);

TEST(SphericalPositionTest, StoresValidCoordinates)
{
    const math::SphericalDirection direction(-180_deg, 90_deg);
    const math::SphericalPosition point(2_km, direction);

    EXPECT_DOUBLE_EQ(point.radius.km(), 2.0);
    EXPECT_DOUBLE_EQ(point.direction.azimuth.deg(), 180.0);
    EXPECT_DOUBLE_EQ(point.direction.elevation.deg(), 90.0);

    const math::SphericalPosition origin(0_m, math::SphericalDirection(-180_deg, -90_deg));
    EXPECT_DOUBLE_EQ(origin.radius.m(), 0.0);
}

TEST(SphericalPositionTest, RejectsNegativeRadius)
{
    EXPECT_THROW((math::SphericalPosition(-1_m, math::SphericalDirection(0_deg, 0_deg))), std::invalid_argument);
    EXPECT_THROW((math::SphericalPosition(
        math::Length::fromMeters(-std::numeric_limits<double>::denorm_min()),
        math::SphericalDirection(0_deg, 0_deg))), std::invalid_argument);
}

TEST(SphericalDirectionTest, NormalizesAzimuthToUnsignedRange)
{
    EXPECT_DOUBLE_EQ(math::SphericalDirection(-30_deg, 0_deg).azimuth.deg(), 330.0);
    EXPECT_DOUBLE_EQ(math::SphericalDirection(180_deg, 0_deg).azimuth.deg(), 180.0);
    EXPECT_DOUBLE_EQ(math::SphericalDirection(360_deg, 0_deg).azimuth.deg(), 0.0);
    EXPECT_DOUBLE_EQ(math::SphericalDirection(540_deg, 0_deg).azimuth.deg(), 180.0);
}

TEST(SphericalDirectionTest, RejectsElevationOutsideInclusiveRange)
{
    EXPECT_THROW((math::SphericalDirection(0_deg, -90.001_deg)), std::invalid_argument);
    EXPECT_THROW((math::SphericalDirection(0_deg, 90.001_deg)), std::invalid_argument);
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
    EXPECT_NEAR(restored.azimuth.deg(), 30.0, 1e-12);
    EXPECT_NEAR(restored.elevation.deg(), 40.0, 1e-12);
}

TEST(SphericalDirectionTest, ConvertsVerticalCartesianUnitVectors)
{
    const math::SphericalDirection up = math::SphericalDirection::fromCartesianUnitVector(
        math::Vector{0.0, 0.0, 2.0});
    EXPECT_DOUBLE_EQ(up.azimuth.deg(), 0.0);
    EXPECT_DOUBLE_EQ(up.elevation.deg(), 90.0);
}

TEST(SphericalDirectionTest, ConvertsLargeFiniteCartesianVectorsWithoutOverflow)
{
    const double max = std::numeric_limits<double>::max();
    const math::SphericalDirection direction =
        math::SphericalDirection::fromCartesianUnitVector(math::Vector{max, max, max});

    EXPECT_DOUBLE_EQ(direction.azimuth.deg(), 45.0);
    EXPECT_NEAR(direction.elevation.deg(), 35.264389682754654, 1e-12);
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

TEST(CartesianGeometryTest, StoresCoordinatesAndSupportsCopyConstruction)
{
    const math::CartesianPosition point(-1_m, 2_m, 3_m);
    const math::CartesianPosition pointCopy(point);
    EXPECT_EQ(pointCopy.x, -1_m);
    EXPECT_EQ(pointCopy.y, 2_m);
    EXPECT_EQ(pointCopy.z, 3_m);

    const math::CartesianVelocity velocity(-1_mps, 2_mps, 3_mps);
    const math::CartesianVelocity velocityCopy(velocity);
    EXPECT_EQ(velocityCopy.x, -1_mps);
    EXPECT_EQ(velocityCopy.y, 2_mps);
    EXPECT_EQ(velocityCopy.z, 3_mps);
}

TEST(CartesianGeometryTest, StoresFiniteBoundaryValues)
{
    const double MAX_VALUE = std::numeric_limits<double>::max();
    const math::CartesianPosition point(
        math::Length::fromMeters(-MAX_VALUE),
        0_m,
        math::Length::fromMeters(MAX_VALUE));
    EXPECT_DOUBLE_EQ(point.x.m(), -MAX_VALUE);
    EXPECT_DOUBLE_EQ(point.y.m(), 0.0);
    EXPECT_DOUBLE_EQ(point.z.m(), MAX_VALUE);

    const math::CartesianVelocity velocity(
        math::LinearVelocity::fromMetersPerSecond(-MAX_VALUE),
        0_mps,
        math::LinearVelocity::fromMetersPerSecond(MAX_VALUE));
    EXPECT_DOUBLE_EQ(velocity.x.mps(), -MAX_VALUE);
    EXPECT_DOUBLE_EQ(velocity.y.mps(), 0.0);
    EXPECT_DOUBLE_EQ(velocity.z.mps(), MAX_VALUE);
}

TEST(CartesianGeometryTest, CalculatesHorizontalAndThreeDimensionalDistance)
{
    const math::CartesianPosition point(3_m, 4_m, 12_m);

    EXPECT_EQ(point.horizontalDistance(), 5_m);
    EXPECT_EQ(point.distance(), 13_m);
}

TEST(CartesianGeometryTest, CalculatesHorizontalAndThreeDimensionalSpeed)
{
    const math::CartesianVelocity velocity(3_mps, 4_mps, 12_mps);

    EXPECT_EQ(velocity.horizontalSpeed(), 5_mps);
    EXPECT_EQ(velocity.speed(), 13_mps);
}
