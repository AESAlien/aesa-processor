#include <beam/domain/beam_width.hpp>

#include <math/quantity/angle.hpp>

#include <gtest/gtest.h>

namespace
{
using namespace math::literals;

// 빔 폭은 방위각 폭과 고각 폭을 독립적으로 보관한다.
TEST(BeamWidthTest, KeepsAzimuthAndElevationWidthsIndependent)
{
    // Given: 서로 다른 두 축의 각도 폭을 준비한다.
    const math::Angle azimuth = 3_deg;
    const math::Angle elevation = 5_deg;

    // When: 두 각도로 빔 폭을 생성한다.
    const beam::BeamWidth width(azimuth, elevation);

    // Then: 각 축의 폭이 전달한 값으로 보관된다.
    EXPECT_EQ(width.azimuth, azimuth);
    EXPECT_EQ(width.elevation, elevation);
}

}
