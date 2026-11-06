#include "math/geometry/cartesian_position.hpp"
#include "math/geometry/cartesian_velocity.hpp"
#include "math/geometry/spherical_position.hpp"
#include "math/trigonometry.hpp"
#include "math/linear_algebra/vector.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>

using namespace math::literals;

static_assert(std::is_same_v<
    decltype(math::CartesianPosition::x),
    math::Length>);
static_assert(std::is_same_v<
    decltype(math::CartesianPosition::y),
    math::Length>);
static_assert(std::is_same_v<
    decltype(math::CartesianPosition::z),
    math::Length>);
static_assert(std::is_same_v<
    decltype(std::declval<const math::CartesianPosition&>().distance()),
    math::Distance>);
static_assert(std::is_same_v<
    decltype(std::declval<const math::CartesianPosition&>().horizontalDistance()),
    math::Distance>);
static_assert(std::is_copy_constructible_v<math::CartesianPosition>);
static_assert(std::is_copy_assignable_v<math::CartesianPosition>);
static_assert(std::is_move_assignable_v<math::CartesianPosition>);
static_assert(!std::is_aggregate_v<math::CartesianPosition>);
static_assert(std::is_copy_constructible_v<math::CartesianVelocity>);
static_assert(std::is_copy_assignable_v<math::CartesianVelocity>);
static_assert(std::is_move_assignable_v<math::CartesianVelocity>);
static_assert(!std::is_aggregate_v<math::CartesianVelocity>);
static_assert(std::is_copy_constructible_v<math::SphericalDirection>);
static_assert(std::is_copy_assignable_v<math::SphericalDirection>);
static_assert(std::is_move_assignable_v<math::SphericalDirection>);
static_assert(!std::is_aggregate_v<math::SphericalDirection>);
static_assert(std::is_copy_constructible_v<math::SphericalPosition>);
static_assert(std::is_copy_assignable_v<math::SphericalPosition>);
static_assert(std::is_move_assignable_v<math::SphericalPosition>);
static_assert(!std::is_aggregate_v<math::SphericalPosition>);
static_assert(!std::is_constructible_v<
    math::SphericalPosition,
    math::Length,
    math::SphericalDirection>);
static_assert(std::is_same_v<
    decltype((std::declval<const math::SphericalPosition&>().radius)),
    const math::Distance&>);



// 객체 생성과 상태 보존 규칙을 검증한다.
TEST(SphericalPositionTest, StoresValidCoordinates)
{
    const math::SphericalDirection direction(-180_deg, 90_deg);
    const math::SphericalPosition point{math::Distance(2_km), direction};

    EXPECT_DOUBLE_EQ(point.radius.km(), 2.0);
    EXPECT_DOUBLE_EQ(point.direction.azimuth.deg(), 180.0);
    EXPECT_DOUBLE_EQ(point.direction.elevation.deg(), 90.0);

    const math::SphericalPosition origin{0_m, -180_deg, -90_deg};
    EXPECT_DOUBLE_EQ(origin.radius.m(), 0.0);
}

// 객체 생성과 상태 보존 규칙을 검증한다.
TEST(CartesianPositionAndVelocityTest, StoresCoordinatesAndSupportsCopyConstruction)
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

// 객체 생성과 상태 보존 규칙을 검증한다.
TEST(CartesianPositionAndVelocityTest, StoresFiniteBoundaryValues)
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

// 값의 변환과 정규화 결과를 검증한다.
TEST(SphericalPositionTest, ConvertsAzimuthUsingPositiveYAxisAsZero)
{
    const math::CartesianPosition forward =
        math::SphericalPosition{10_m, 0_deg, 0_deg}.toCartesian();
    EXPECT_DOUBLE_EQ(forward.x.m(), 0.0);
    EXPECT_DOUBLE_EQ(forward.y.m(), 10.0);
    EXPECT_DOUBLE_EQ(forward.z.m(), 0.0);

    const math::CartesianPosition right =
        math::SphericalPosition{10_m, 90_deg, 0_deg}.toCartesian();
    EXPECT_NEAR(right.x.m(), 10.0, 1e-12);
    EXPECT_NEAR(right.y.m(), 0.0, 1e-12);
    EXPECT_DOUBLE_EQ(right.z.m(), 0.0);
}

// 값의 변환과 정규화 결과를 검증한다.
TEST(SphericalPositionTest, ConvertsZeroRadiusToOrigin)
{
    const math::CartesianPosition origin =
        math::SphericalPosition{0_m, 123_deg, -45_deg}.toCartesian();

    EXPECT_DOUBLE_EQ(origin.x.m(), 0.0);
    EXPECT_DOUBLE_EQ(origin.y.m(), 0.0);
    EXPECT_DOUBLE_EQ(origin.z.m(), 0.0);
}

// 값의 변환과 정규화 결과를 검증한다.
TEST(SphericalPositionTest, RoundTripsPositionAcrossAzimuthQuadrants)
{
    for (double azimuthDegrees : {30.0, 120.0, 210.0, 300.0, -30.0})
    {
        const math::SphericalPosition original{
            2_km,
                math::Angle::fromDegrees(azimuthDegrees),
                -20_deg};
        const math::CartesianPosition cartesian = original.toCartesian();
        const math::SphericalPosition restored = cartesian.toSpherical();

        EXPECT_NEAR(restored.radius.m(), original.radius.m(), 1e-9);
        EXPECT_NEAR(restored.direction.azimuth.deg(), original.direction.azimuth.deg(), 1e-12);
        EXPECT_NEAR(restored.direction.elevation.deg(), original.direction.elevation.deg(), 1e-12);
    }
}

// 값의 변환과 정규화 결과를 검증한다.
TEST(CartesianPositionTest, ConvertsKnownPositionToSphericalCoordinates)
{
    const math::SphericalPosition position = math::CartesianPosition(3_m, 4_m, 12_m).toSpherical();

    EXPECT_DOUBLE_EQ(position.radius.m(), 13.0);
    EXPECT_NEAR(position.direction.azimuth.deg(), 36.86989764584402, 1e-12);
    EXPECT_NEAR(position.direction.elevation.deg(), 67.38013505195957, 1e-12);
}

// 값의 변환과 정규화 결과를 검증한다.
TEST(CartesianPositionTest, ConvertsVerticalPositionsToSphericalCoordinates)
{
    for (double heightMeters : {-2.0, 2.0})
    {
        const math::SphericalPosition position = math::CartesianPosition(
            0_m, 0_m, math::Length::fromMeters(heightMeters)).toSpherical();

        EXPECT_DOUBLE_EQ(position.radius.m(), 2.0);
        EXPECT_DOUBLE_EQ(position.direction.azimuth.deg(), 0.0);
        EXPECT_DOUBLE_EQ(position.direction.elevation.deg(), heightMeters > 0.0 ? 90.0 : -90.0);
    }
}

// 값의 변환과 정규화 결과를 검증한다.
TEST(CartesianPositionTest, ConvertsExtremeFinitePositionScales)
{
    for (double scale : {1e-200, 1e200})
    {
        const math::CartesianPosition original(
            3_m * scale,
            -4_m * scale,
            12_m * scale);
        const math::SphericalPosition spherical = original.toSpherical();
        const math::CartesianPosition restored = spherical.toCartesian();

        EXPECT_NEAR(spherical.radius.m() / scale, 13.0, 1e-12);
        EXPECT_NEAR(restored.x.m() / scale, 3.0, 1e-12);
        EXPECT_NEAR(restored.y.m() / scale, -4.0, 1e-12);
        EXPECT_NEAR(restored.z.m() / scale, 12.0, 1e-12);
    }
}

// 값의 변환과 정규화 결과를 검증한다.
TEST(SphericalDirectionTest, NormalizesAzimuthToUnsignedRange)
{
    EXPECT_DOUBLE_EQ(math::SphericalDirection(-30_deg, 0_deg).azimuth.deg(), 330.0);
    EXPECT_DOUBLE_EQ(math::SphericalDirection(180_deg, 0_deg).azimuth.deg(), 180.0);
    EXPECT_DOUBLE_EQ(math::SphericalDirection(360_deg, 0_deg).azimuth.deg(), 0.0);
    EXPECT_DOUBLE_EQ(math::SphericalDirection(540_deg, 0_deg).azimuth.deg(), 180.0);
}

// 값의 변환과 정규화 결과를 검증한다.
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

// 값의 변환과 정규화 결과를 검증한다.
TEST(SphericalDirectionTest, ConvertsVerticalCartesianUnitVectors)
{
    const math::SphericalDirection up = math::SphericalDirection::fromCartesianUnitVector(
        math::Vector{0.0, 0.0, 2.0});
    EXPECT_DOUBLE_EQ(up.azimuth.deg(), 0.0);
    EXPECT_DOUBLE_EQ(up.elevation.deg(), 90.0);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(CartesianPositionTest, CalculatesHorizontalAndThreeDimensionalDistance)
{
    const math::CartesianPosition point(3_m, 4_m, 12_m);

    EXPECT_EQ(point.horizontalDistance(), 5_m);
    EXPECT_EQ(point.distance(), 13_m);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(CartesianVelocityTest, CalculatesHorizontalAndThreeDimensionalSpeed)
{
    const math::CartesianVelocity velocity(3_mps, 4_mps, 12_mps);

    EXPECT_EQ(velocity.horizontalSpeed(), 5_mps);
    EXPECT_EQ(velocity.speed(), 13_mps);
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(SphericalPositionTest, ComparesWithDefaultTolerance)
{
    const math::SphericalPosition point{10_m, 20_deg, 30_deg};
    const math::SphericalPosition nearby{
        math::Length::fromMeters(10.0 + 1e-10),
            math::Angle::fromDegrees(20.0 + 1e-10),
            30_deg};

    EXPECT_TRUE(point.equals(nearby));
    EXPECT_TRUE(point == nearby);
    EXPECT_FALSE(point != nearby);
    EXPECT_FALSE(point.equals(nearby, math::Length::NONE_TOLERANCE, math::Angle::NONE_TOLERANCE));
    EXPECT_TRUE(point.equals(point, math::Length::NONE_TOLERANCE, math::Angle::NONE_TOLERANCE));
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(SphericalPositionTest, ComparesAzimuthAcrossWrapBoundary)
{
    const math::SphericalPosition left{10_m, 359.999_deg, 0_deg};
    const math::SphericalPosition right{10_m, 0.001_deg, 0_deg};
    const math::QuantityTolerance<math::Angle> tolerance{0.003_deg, 0.0};

    EXPECT_TRUE(left.equals(right, math::Length::NONE_TOLERANCE, tolerance));
    EXPECT_TRUE(right.equals(left, math::Length::NONE_TOLERANCE, tolerance));
    EXPECT_FALSE(left.equals(right, math::Length::NONE_TOLERANCE, math::Angle::NONE_TOLERANCE));
    EXPECT_EQ((math::SphericalPosition{10_m, -1_deg, 0_deg}),
        (math::SphericalPosition{10_m, 359_deg, 0_deg}));
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(SphericalPositionTest, ComparesEachCoordinateWithSeparateTolerance)
{
    const math::SphericalPosition point{10_m, 20_deg, 30_deg};
    const math::QuantityTolerance<math::Length> rangeTolerance{1_m, 0.0};
    const math::QuantityTolerance<math::Angle> angleTolerance{0.1_deg, 0.0};
    const math::SphericalPosition nearby{10.5_m, 20.05_deg, 30.05_deg};

    EXPECT_TRUE(point.equals(nearby, rangeTolerance, angleTolerance));
    EXPECT_FALSE(point.equals(math::SphericalPosition{math::Distance(12_m), point.direction}, rangeTolerance, angleTolerance));
    EXPECT_FALSE(point.equals(math::SphericalPosition{10_m, 21_deg, 30_deg},
        rangeTolerance, angleTolerance));
    EXPECT_FALSE(point.equals(math::SphericalPosition{10_m, 20_deg, 31_deg},
        rangeTolerance, angleTolerance));
    EXPECT_TRUE(point != nearby);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(CartesianPositionTest, RejectsOriginWithoutSphericalDirection)
{
    const math::CartesianPosition origin(0_m, 0_m, 0_m);

    EXPECT_THROW(origin.toSpherical(), std::invalid_argument);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(CartesianPositionTest, RejectsUnrepresentableSphericalRadius)
{
    const auto maximum = math::Length::fromMeters(std::numeric_limits<double>::max());
    const math::CartesianPosition position(maximum, maximum, maximum);

    EXPECT_THROW(position.toSpherical(), std::overflow_error);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(SphericalDirectionTest, RejectsElevationOutsideInclusiveRange)
{
    EXPECT_THROW((math::SphericalDirection(0_deg, -90.001_deg)), std::invalid_argument);
    EXPECT_THROW((math::SphericalDirection(0_deg, 90.001_deg)), std::invalid_argument);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(SphericalDirectionTest, ConvertsLargeFiniteCartesianVectorsWithoutOverflow)
{
    const double max = std::numeric_limits<double>::max();
    const math::SphericalDirection direction =
        math::SphericalDirection::fromCartesianUnitVector(math::Vector{max, max, max});

    EXPECT_DOUBLE_EQ(direction.azimuth.deg(), 45.0);
    EXPECT_NEAR(direction.elevation.deg(), 35.264389682754654, 1e-12);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
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
