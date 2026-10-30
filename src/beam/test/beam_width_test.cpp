#include <beam/domain/beam_width.hpp>

#include <math/quantity/angle.hpp>

#include <gtest/gtest.h>

namespace
{
using namespace math::literals;

TEST(BeamWidthTest, KeepsAzimuthAndElevationWidthsIndependent)
{
    const beam::BeamWidth width(3_deg, 5_deg);

    EXPECT_EQ(width.azimuth, 3_deg);
    EXPECT_EQ(width.elevation, 5_deg);
}

}
