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

static_assert(std::is_const_v<decltype(math::CartesianDisplacement::x)>);
static_assert(std::is_const_v<decltype(math::CartesianDisplacement::y)>);
static_assert(std::is_const_v<decltype(math::CartesianDisplacement::z)>);
static_assert(std::is_copy_constructible_v<math::CartesianDisplacement>);
static_assert(!std::is_copy_assignable_v<math::CartesianDisplacement>);
static_assert(!std::is_move_assignable_v<math::CartesianDisplacement>);
static_assert(!std::is_aggregate_v<math::CartesianDisplacement>);

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

TEST(CartesianDisplacementTest, SubtractsIdenticalPositionsToZero)
{
    const math::CartesianPosition point(1_m, -2_m, 3_m);
    const auto displacement = point - point;

    EXPECT_EQ(displacement.distance(), 0_m);
    EXPECT_EQ(displacement.toVelocity(std::chrono::seconds{1}).speed(), 0_mps);
}

TEST(CartesianDisplacementTest, ExportsMeterComponentsInAxisOrder)
{
    const math::CartesianDisplacement displacement(1_km, -2_m, 3_m);
    const auto vector = displacement.toVector();

    ASSERT_EQ(vector.size(), 3);
    EXPECT_DOUBLE_EQ(vector[0], 1000.0);
    EXPECT_DOUBLE_EQ(vector[1], -2.0);
    EXPECT_DOUBLE_EQ(vector[2], 3.0);
}

TEST(CartesianDisplacementTest, CalculatesVelocityFromElapsedSeconds)
{
    const math::CartesianDisplacement displacement(3_m, -4_m, 12_m);
    const auto velocity = displacement.toVelocity(std::chrono::milliseconds{500});

    EXPECT_EQ(velocity.x, 6_mps);
    EXPECT_EQ(velocity.y, -8_mps);
    EXPECT_EQ(velocity.z, 24_mps);
    EXPECT_EQ(velocity.speed(), 26_mps);
}

TEST(CartesianDisplacementTest, RejectsInvalidElapsedTime)
{
    const math::CartesianDisplacement displacement(1_m, 0_m, 0_m);
    for (double seconds : {0.0, -1.0, std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
    {
        EXPECT_THROW(displacement.toVelocity(std::chrono::duration<double>{seconds}), std::invalid_argument);
    }
}

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

TEST(CartesianDisplacementTest, RejectsVelocityOverflow)
{
    const math::CartesianDisplacement displacement(1_m, 0_m, 0_m);

    EXPECT_THROW(displacement.toVelocity(
        std::chrono::duration<double>{std::numeric_limits<double>::denorm_min()}), std::invalid_argument);
}

TEST(CartesianDisplacementTest, RejectsSubtractionOverflow)
{
    const auto maximum = math::Length::fromMeters(std::numeric_limits<double>::max());
    const math::CartesianPosition positive(maximum, 0_m, 0_m);
    const math::CartesianPosition negative(-maximum, 0_m, 0_m);

    EXPECT_THROW(positive - negative, std::invalid_argument);
}

TEST(CartesianDisplacementTest, CalculatesDistanceAtExtremeFiniteScales)
{
    for (double scale : {1e-200, 1e200})
    {
        const math::CartesianDisplacement displacement(
            math::Length::fromMeters(3.0 * scale),
            math::Length::fromMeters(-4.0 * scale),
            math::Length::fromMeters(12.0 * scale));

        EXPECT_NEAR(displacement.distance().m() / scale, 13.0, 1e-12);
    }
}
