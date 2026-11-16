#include <pose/domain/enu_geo_transform.hpp>

#include <gtest/gtest.h>

using namespace math::literals;

TEST(EnuGeoTransformTest, ReturnsOriginAndAppliesUpAlongTheEllipsoidNormal)
{
    const auto radar = pose::GeoPosition(math::TiltAngle(37_deg), math::CircularAngle(127_deg), 100_m);
    const auto transform = pose::EnuGeoTransform(radar);

    const auto origin = transform.enuToGeo(math::CartesianPosition(0_m, 0_m, 0_m));
    EXPECT_EQ(origin.latitude, radar.latitude);
    EXPECT_EQ(origin.longitude, radar.longitude);
    EXPECT_EQ(origin.altitude, radar.altitude);

    const auto above = transform.enuToGeo(math::CartesianPosition(0_m, 0_m, 250_m));
    EXPECT_NEAR(above.latitude.deg(), 37.0, 1e-10);
    EXPECT_NEAR(above.longitude.deg(), 127.0, 1e-10);
    EXPECT_NEAR(above.altitude.m(), 350.0, 1e-8);
}

TEST(EnuGeoTransformTest, EastwardOffsetCrossesDateLineAndRaisesEllipsoidalHeight)
{
    const auto radar = pose::GeoPosition(math::TiltAngle(0_deg), math::CircularAngle(179.9_deg), 0_m);
    const auto transform = pose::EnuGeoTransform(radar);

    const auto result = transform.enuToGeo(math::CartesianPosition(100_km, 0_m, 0_m));

    EXPECT_NEAR(result.latitude.deg(), 0.0, 1e-10);
    EXPECT_NEAR(result.longitude.deg(), 180.79824168785, 1e-10);
    EXPECT_NEAR(result.altitude.m(), 783.8798016, 1e-6);
}

TEST(EnuGeoTransformTest, GeoToEnuRestoresLocalCoordinates)
{
    const auto origin = pose::GeoPosition(math::TiltAngle(37_deg), math::CircularAngle(127_deg), 100_m);
    const auto transform = pose::EnuGeoTransform(origin);
    const auto position_enu = math::CartesianPosition(12_km, -30_km, 200_m);

    const auto restored_enu = transform.geoToEnu(transform.enuToGeo(position_enu));

    EXPECT_NEAR(restored_enu.x.m(), position_enu.x.m(), 1e-6);
    EXPECT_NEAR(restored_enu.y.m(), position_enu.y.m(), 1e-6);
    EXPECT_NEAR(restored_enu.z.m(), position_enu.z.m(), 1e-6);
}
