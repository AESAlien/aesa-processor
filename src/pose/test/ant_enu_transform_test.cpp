#include <pose/domain/ant_enu_transform.hpp>
#include <pose/domain/factory/radar_attitude_factory.hpp>

#include <gtest/gtest.h>

#include <math/trigonometry.hpp>

using namespace math::literals;

namespace
{

pose::RadarAttitude makeRadarAttitude(math::Angle roll, math::Angle pitch, math::Angle yaw)
{
    const auto factory = pose::RadarAttitudeFactory(pose::RadarAttitudeFactoryConfig(89_deg));
    return factory.create(roll, math::TiltAngle(pitch), yaw);
}

pose::AntEnuTransform makeTransform(math::Angle roll, math::Angle pitch, math::Angle yaw)
{
    return pose::AntEnuTransform(
        makeRadarAttitude(roll, pitch, yaw),
        pose::AntEnuTransformConfig(math::SquareMatrix::identity(3))
    );
}

} // namespace

// 영 자세 변환은 입력 방향의 방위각과 고각을 유지한다.
TEST(AntEnuTransformTest, ZeroAttitudeKeepsAngles)
{
    // Given: 영 자세의 좌표 변환을 준비한다.
    const auto transform = makeTransform(0_deg, 0_deg, 0_deg);

    // When: 안테나 방향을 ENU 방향으로 변환한다.
    const auto direction_enu = transform.antToEnu(math::SphericalDirection(-30.0_deg, 10.0_deg));

    // Then: 영 자세 변환은 입력 방향의 방위각과 고각을 유지한다.
    EXPECT_DOUBLE_EQ(direction_enu.azimuth.deg(), 330.0);
    EXPECT_EQ(direction_enu.elevation, 10_deg);
}

// 좌표 변환은 장착 회전을 안테나 방향에 반영한다.
TEST(AntEnuTransformTest, MountRotationIsAppliedToAntDirection)
{
    // Given: 90도 장착 회전이 있는 좌표 변환을 준비한다.
    const auto mount = math::SquareMatrix({{0, -1, 0}, {1, 0, 0}, {0, 0, 1}});
    const auto config = pose::AntEnuTransformConfig(mount);
    const auto transform = pose::AntEnuTransform(makeRadarAttitude(0_deg, 0_deg, 0_deg), config);

    // When: 안테나 정면을 ENU 방향으로 변환한다.
    const auto direction_enu = transform.antToEnu(math::SphericalDirection(0_deg, 0_deg));

    // Then: 좌표 변환은 장착 회전을 안테나 방향에 반영한다.
    EXPECT_TRUE(direction_enu.azimuth.isEquivalentTo(-90_deg));
    EXPECT_EQ(direction_enu.elevation, 0_deg);
}

// 요 회전은 방위각을 시계 방향으로 회전시킨다.
TEST(AntEnuTransformTest, YawRotatesAzimuthClockwise)
{
    // Given: 요가 90도인 좌표 변환을 준비한다.
    const auto transform = makeTransform(0_deg, 0_deg, 90_deg); // 정면이 동쪽

    // When: 안테나 정면을 ENU 방향으로 변환한다.
    const auto direction_enu = transform.antToEnu(math::SphericalDirection(0_deg, 0_deg));

    // Then: 요 회전은 방위각을 시계 방향으로 회전시킨다.
    EXPECT_TRUE(direction_enu.azimuth.isEquivalentTo(90_deg));
    EXPECT_EQ(direction_enu.elevation, 0_deg);
}

// 피치 회전은 안테나 정면의 고각을 올린다.
TEST(AntEnuTransformTest, PitchRaisesBoresight)
{
    // Given: 피치가 10도인 좌표 변환을 준비한다.
    const auto transform = makeTransform(0_deg, 10_deg, 0_deg);

    // When: 안테나 정면을 ENU 방향으로 변환한다.
    const auto direction_enu = transform.antToEnu(math::SphericalDirection(0_deg, 0_deg));

    // Then: 피치 회전은 안테나 정면의 고각을 올린다.
    EXPECT_TRUE(direction_enu.azimuth.isEquivalentTo(0_deg));
    EXPECT_EQ(direction_enu.elevation, 10_deg);
}

// 정방향 변환 후 역방향 변환은 원래 방향을 복원한다.
TEST(AntEnuTransformTest, AntToEnuAndBackRestoresAngles)
{
    // Given: 세 축으로 회전한 좌표 변환을 준비한다.
    const auto transform = makeTransform(10_deg, 20_deg, 30_deg);

    // When: 여러 방향을 ENU로 변환한 뒤 안테나 방향으로 되돌린다.
    for (const auto azimuth_ant : {-45.0_deg, -30.0_deg, 0.0_deg, 15.0_deg, 45.0_deg})
    {
        for (const auto elevation_ant : {0.0_deg, 10.0_deg, 41.2_deg})
        {
            const auto direction_enu = transform.antToEnu(math::SphericalDirection(azimuth_ant, elevation_ant));
            const auto directionRestored_ant = transform.enuToAnt(direction_enu);

            // Then: 정방향 변환 후 역방향 변환은 원래 방향을 복원한다.
            EXPECT_TRUE(directionRestored_ant.azimuth.isEquivalentTo(math::wrap360(azimuth_ant)));
            EXPECT_EQ(directionRestored_ant.elevation, elevation_ant);
        }
    }
}
