#include "math/geometry/cartesian_acceleration.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <type_traits>

using namespace math::literals;
using namespace std::chrono_literals;

static_assert(std::is_const_v<decltype(math::CartesianAcceleration::x)>);
static_assert(std::is_const_v<decltype(math::CartesianAcceleration::y)>);
static_assert(std::is_const_v<decltype(math::CartesianAcceleration::z)>);
static_assert(std::is_same_v<
    std::remove_const_t<decltype(math::CartesianAcceleration::x)>,
    math::LinearAcceleration>);
static_assert(std::is_copy_constructible_v<math::CartesianAcceleration>);
static_assert(!std::is_copy_assignable_v<math::CartesianAcceleration>);
static_assert(!std::is_move_assignable_v<math::CartesianAcceleration>);
static_assert(!std::is_aggregate_v<math::CartesianAcceleration>);

TEST(CartesianAccelerationTest, StoresCoordinatesAndSupportsCopyConstruction)
{
    const math::CartesianAcceleration acceleration(-1_mps2, 2_mps2, 3_mps2);
    const math::CartesianAcceleration copy(acceleration);

    EXPECT_EQ(copy.x, -1_mps2);
    EXPECT_EQ(copy.y, 2_mps2);
    EXPECT_EQ(copy.z, 3_mps2);
}

TEST(CartesianAccelerationTest, CalculatesVelocityFromElapsedTime)
{
    const math::CartesianAcceleration acceleration(6_mps2, -8_mps2, 24_mps2);
    const math::CartesianVelocity velocity = acceleration * 500ms;

    EXPECT_EQ(velocity.x, 3_mps);
    EXPECT_EQ(velocity.y, -4_mps);
    EXPECT_EQ(velocity.z, 12_mps);
}

TEST(CartesianAccelerationTest, PreservesElapsedTimeSign)
{
    const math::CartesianAcceleration acceleration(1_mps2, -2_mps2, 3_mps2);
    const auto velocity = acceleration * -2s;

    EXPECT_EQ(velocity.x, -2_mps);
    EXPECT_EQ(velocity.y, 4_mps);
    EXPECT_EQ(velocity.z, -6_mps);
    const auto stopped = acceleration * 0s;
    EXPECT_EQ(stopped.x, 0_mps);
    EXPECT_EQ(stopped.y, 0_mps);
    EXPECT_EQ(stopped.z, 0_mps);
}

TEST(CartesianAccelerationTest, CalculatesMagnitude)
{
    const math::CartesianAcceleration acceleration(3_mps2, -4_mps2, 12_mps2);

    EXPECT_EQ(acceleration.horizontalAcceleration(), 5_mps2);
    EXPECT_EQ(acceleration.acceleration(), 13_mps2);

    for (double scale : {1e-200, 1e200})
    {
        const math::CartesianAcceleration scaled(
            math::LinearAcceleration::fromMetersPerSecondSquared(3.0 * scale),
            math::LinearAcceleration::fromMetersPerSecondSquared(4.0 * scale),
            math::LinearAcceleration::fromMetersPerSecondSquared(12.0 * scale));
        EXPECT_NEAR(scaled.horizontalAcceleration().mps2() / scale, 5.0, 1e-12);
        EXPECT_NEAR(scaled.acceleration().mps2() / scale, 13.0, 1e-12);
    }
}

TEST(CartesianAccelerationTest, RejectsInvalidVelocityResult)
{
    const auto maximum = math::LinearAcceleration::fromMetersPerSecondSquared(
        std::numeric_limits<double>::max());
    const math::CartesianAcceleration acceleration(maximum, 0_mps2, 0_mps2);

    EXPECT_THROW(acceleration * 2s, std::invalid_argument);
    EXPECT_THROW(
        acceleration * std::chrono::duration<double>{
            std::numeric_limits<double>::infinity()},
        std::invalid_argument);
    EXPECT_THROW(
        acceleration * std::chrono::duration<double>{
            std::numeric_limits<double>::quiet_NaN()},
        std::invalid_argument);
}
