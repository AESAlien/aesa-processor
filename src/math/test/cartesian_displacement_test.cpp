#include "math/geometry/cartesian_displacement.hpp"
#include "math/geometry/cartesian_position.hpp"
#include "math/geometry/cartesian_velocity.hpp"
#include "math/linear_algebra/vector.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <stdexcept>
#include <type_traits>

using namespace math::literals;

static_assert(std::is_copy_constructible_v<math::CartesianDisplacement>);
static_assert(std::is_copy_assignable_v<math::CartesianDisplacement>);
static_assert(std::is_move_assignable_v<math::CartesianDisplacement>);
static_assert(!std::is_aggregate_v<math::CartesianDisplacement>);



// 값의 변환과 정규화 결과를 검증한다.
TEST(CartesianDisplacementTest, ExportsMeterComponentsInAxisOrder)
{
    const math::CartesianDisplacement displacement(1_km, -2_m, 3_m);
    const auto vector = displacement.toVector();

    ASSERT_EQ(vector.size(), 3);
    EXPECT_DOUBLE_EQ(vector[0], 1000.0);
    EXPECT_DOUBLE_EQ(vector[1], -2.0);
    EXPECT_DOUBLE_EQ(vector[2], 3.0);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(CartesianDisplacementTest, AddsDisplacementToPosition)
{
    const math::CartesianPosition position(1_m, -2_m, 3_m);
    const math::CartesianDisplacement displacement(4_m, -6_m, 15_m);
    const math::CartesianPosition result = position + displacement;

    EXPECT_EQ(result.x, 5_m);
    EXPECT_EQ(result.y, -8_m);
    EXPECT_EQ(result.z, 18_m);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(CartesianDisplacementTest, SubtractsDisplacementFromPosition)
{
    const math::CartesianPosition position(5_m, -8_m, 18_m);
    const math::CartesianDisplacement displacement(4_m, -6_m, 15_m);
    const math::CartesianPosition result = position - displacement;

    EXPECT_EQ(result.x, 1_m);
    EXPECT_EQ(result.y, -2_m);
    EXPECT_EQ(result.z, 3_m);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(CartesianDisplacementTest, AddsDisplacements)
{
    const math::CartesianDisplacement left(1_m, -2_m, 3_m);
    const math::CartesianDisplacement right(4_m, -6_m, 15_m);
    const math::CartesianDisplacement result = left + right;

    EXPECT_EQ(result.x, 5_m);
    EXPECT_EQ(result.y, -8_m);
    EXPECT_EQ(result.z, 18_m);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(CartesianDisplacementTest, CalculatesDisplacementFromVelocityAndElapsedTime)
{
    const math::CartesianVelocity velocity(6_mps, -8_mps, 24_mps);
    const math::CartesianDisplacement displacement = velocity * std::chrono::milliseconds{500};

    EXPECT_EQ(displacement.x, 3_m);
    EXPECT_EQ(displacement.y, -4_m);
    EXPECT_EQ(displacement.z, 12_m);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(CartesianDisplacementTest, PreservesElapsedTimeSign)
{
    const math::CartesianVelocity velocity(6_mps, -8_mps, 24_mps);
    const math::CartesianDisplacement displacement = velocity * std::chrono::seconds{-2};

    EXPECT_EQ(displacement.x, -12_m);
    EXPECT_EQ(displacement.y, 16_m);
    EXPECT_EQ(displacement.z, -48_m);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(CartesianDisplacementTest, SubtractsPositionsInCorrectDirection)
{
    const math::CartesianPosition previous(1_m, -2_m, 3_m);
    const math::CartesianPosition current(4_m, -6_m, 15_m);
    const auto displacement = current - previous;
    const auto opposite = previous - current;

    EXPECT_EQ(displacement.x, 3_m);
    EXPECT_EQ(displacement.y, -4_m);
    EXPECT_EQ(displacement.z, 12_m);
    EXPECT_EQ(opposite.x, -3_m);
    EXPECT_EQ(opposite.y, 4_m);
    EXPECT_EQ(opposite.z, -12_m);
    EXPECT_EQ(displacement.distance(), 13_m);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(CartesianDisplacementTest, SubtractsIdenticalPositionsToZero)
{
    const math::CartesianPosition point(1_m, -2_m, 3_m);
    const auto displacement = point - point;

    EXPECT_EQ(displacement.distance(), 0_m);
    EXPECT_EQ(displacement.toVelocity(std::chrono::seconds{1}).speed(), 0_mps);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(CartesianDisplacementTest, CalculatesVelocityFromElapsedSeconds)
{
    const math::CartesianDisplacement displacement(3_m, -4_m, 12_m);
    const auto velocity = displacement.toVelocity(std::chrono::milliseconds{500});

    EXPECT_EQ(velocity.x, 6_mps);
    EXPECT_EQ(velocity.y, -8_mps);
    EXPECT_EQ(velocity.z, 24_mps);
    EXPECT_EQ(velocity.speed(), 26_mps);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(CartesianDisplacementTest, CalculatesDistanceAtExtremeFiniteScales)
{
    for (double scale : {1e-200, 1e200})
    {
        const math::CartesianDisplacement displacement(
            3_m * scale,
            -4_m * scale,
            12_m * scale);

        EXPECT_NEAR(displacement.distance().m() / scale, 13.0, 1e-12);
    }
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(CartesianDisplacementTest, RejectsAdditionOverflow)
{
    const auto maximum = math::Length::fromMeters(std::numeric_limits<double>::max());
    const math::CartesianPosition position(maximum, 0_m, 0_m);
    const math::CartesianDisplacement displacement(maximum, 0_m, 0_m);

    EXPECT_THROW(position + displacement, std::invalid_argument);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(CartesianDisplacementTest, RejectsPositionDisplacementSubtractionOverflow)
{
    const auto maximum = math::Length::fromMeters(std::numeric_limits<double>::max());
    const math::CartesianPosition position(-maximum, 0_m, 0_m);
    const math::CartesianDisplacement displacement(maximum, 0_m, 0_m);

    EXPECT_THROW(position - displacement, std::invalid_argument);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(CartesianDisplacementTest, RejectsDisplacementAdditionOverflow)
{
    const auto maximum = math::Length::fromMeters(std::numeric_limits<double>::max());
    const math::CartesianDisplacement left(maximum, 0_m, 0_m);
    const math::CartesianDisplacement right(maximum, 0_m, 0_m);

    EXPECT_THROW(left + right, std::invalid_argument);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(CartesianDisplacementTest, RejectsInvalidVelocityTimeProduct)
{
    const auto maximum = math::LinearVelocity::fromMetersPerSecond(
        std::numeric_limits<double>::max());
    const math::CartesianVelocity velocity(maximum, 0_mps, 0_mps);

    EXPECT_THROW(velocity * std::chrono::seconds{2}, std::invalid_argument);
    EXPECT_THROW(velocity * std::chrono::duration<double>{
        std::numeric_limits<double>::quiet_NaN()}, std::invalid_argument);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(CartesianDisplacementTest, RejectsInvalidElapsedTime)
{
    const math::CartesianDisplacement displacement(1_m, 0_m, 0_m);
    for (double seconds : {0.0, -1.0, std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
    {
        EXPECT_THROW(displacement.toVelocity(std::chrono::duration<double>{seconds}), std::invalid_argument);
    }
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(CartesianDisplacementTest, DividesSubnormalComponentsWithoutReciprocalOverflow)
{
    const double smallest = std::numeric_limits<double>::denorm_min();
    const math::CartesianDisplacement displacement(
        math::Length::fromMeters(smallest), math::Length::fromMeters(-smallest), 0_m);
    const auto velocity = displacement.toVelocity(std::chrono::duration<double>{smallest});

    EXPECT_EQ(velocity.x, 1_mps);
    EXPECT_EQ(velocity.y, -1_mps);
    EXPECT_EQ(velocity.z, 0_mps);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(CartesianDisplacementTest, RejectsVelocityOverflow)
{
    const math::CartesianDisplacement displacement(1_m, 0_m, 0_m);

    EXPECT_THROW(displacement.toVelocity(
        std::chrono::duration<double>{std::numeric_limits<double>::denorm_min()}), std::invalid_argument);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(CartesianDisplacementTest, RejectsSubtractionOverflow)
{
    const auto maximum = math::Length::fromMeters(std::numeric_limits<double>::max());
    const math::CartesianPosition positive(maximum, 0_m, 0_m);
    const math::CartesianPosition negative(-maximum, 0_m, 0_m);

    EXPECT_THROW(positive - negative, std::invalid_argument);
}
