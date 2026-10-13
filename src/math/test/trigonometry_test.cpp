#include "math/trigonometry.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

using namespace math::literals;

TEST(TrigonometryTest, ComputesSineCosineAndTangentFromAngles)
{
    EXPECT_DOUBLE_EQ(math::sin(0_deg), 0.0);
    EXPECT_NEAR(math::sin(30_deg), 0.5, 1e-15);
    EXPECT_NEAR(math::sin(-90_deg), -1.0, 1e-15);
    EXPECT_NEAR(math::cos(60_deg), 0.5, 1e-15);
    EXPECT_DOUBLE_EQ(math::cos(0_deg), 1.0);
    EXPECT_NEAR(math::tan(45_deg), 1.0, 1e-15);
    EXPECT_NEAR(math::tan(-45_deg), -1.0, 1e-15);
}

TEST(TrigonometryTest, RejectsNonfiniteAngles)
{
    const auto infinity = math::Angle::fromDegrees(std::numeric_limits<double>::infinity());
    const auto nan = math::Angle::fromDegrees(std::numeric_limits<double>::quiet_NaN());

    EXPECT_THROW(math::sin(infinity), std::domain_error);
    EXPECT_THROW(math::cos(nan), std::domain_error);
    EXPECT_THROW(math::tan(infinity), std::domain_error);
}

TEST(TrigonometryTest, ComputesInverseSineCosineAndTangent)
{
    EXPECT_NEAR(math::asin(0.5).deg(), 30.0, 1e-12);
    EXPECT_NEAR(math::asin(-1.0).deg(), -90.0, 1e-12);
    EXPECT_NEAR(math::acos(0.5).deg(), 60.0, 1e-12);
    EXPECT_NEAR(math::acos(-1.0).deg(), 180.0, 1e-12);
    EXPECT_NEAR(math::atan(1.0).deg(), 45.0, 1e-12);
    EXPECT_NEAR(math::atan(-1.0).deg(), -45.0, 1e-12);
}

TEST(TrigonometryTest, RejectsInvalidInverseFunctionInputs)
{
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();

    EXPECT_THROW(math::asin(-1.001), std::domain_error);
    EXPECT_THROW(math::asin(1.001), std::domain_error);
    EXPECT_THROW(math::acos(infinity), std::domain_error);
    EXPECT_THROW(math::acos(nan), std::domain_error);
    EXPECT_THROW(math::atan(infinity), std::domain_error);
    EXPECT_THROW(math::atan(nan), std::domain_error);
}
