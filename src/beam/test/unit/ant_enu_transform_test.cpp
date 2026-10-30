#include "attitude/ant_enu_transform.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <math/trigonometry.hpp>

using namespace math::literals;

namespace
{

beam::RadarAttitude makeAttitude(math::Angle roll, math::Angle pitch, math::Angle yaw)
{
    return beam::RadarAttitude(roll, math::TiltAngle(pitch), yaw);
}

beam::AntEnuTransform makeValidTransform(math::Angle roll, math::Angle pitch, math::Angle yaw)
{
    return beam::AntEnuTransform(
        makeAttitude(roll, pitch, yaw),
        beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}));
}

} // namespace

// ---------------------------------------------------------------- makeAntToEnu

// 영 자세 변환은 입력 방향의 방위각과 고각을 유지한다.
TEST(AttitudeTransformTest, ZeroAttitudeKeepsAngles)
{
    // Given: 영 자세의 좌표 변환을 준비한다.
    const beam::AntEnuTransform transform = makeValidTransform(0_deg, 0_deg, 0_deg);

    // When: 안테나 방향을 ENU 방향으로 변환한다.
    const auto direction_enu = transform.antToEnu(math::SphericalDirection(-30.0_deg, 10.0_deg));

    // Then: 영 자세 변환은 입력 방향의 방위각과 고각을 유지한다.
    EXPECT_DOUBLE_EQ(direction_enu.azimuth.deg(), 330.0);
    EXPECT_EQ(direction_enu.elevation, 10_deg);
}

// 좌표 변환은 라디안으로 생성한 자세와 방향을 처리한다.
TEST(AttitudeTransformTest, AcceptsAnglesCreatedFromRadians)
{
    // Given: 라디안으로 지정한 요 회전을 준비한다.
    const auto transform = makeValidTransform(0_deg, 0_deg, math::Angle::fromRadians(math::PI / 2.0));

    // When: 라디안으로 지정한 안테나 방향을 변환한다.
    const auto direction_enu = transform.antToEnu(math::SphericalDirection(
        math::Angle::fromRadians(-math::PI / 6.0),
        math::Angle::fromRadians(math::PI / 18.0)));

    // Then: 좌표 변환은 라디안으로 생성한 자세와 방향을 처리한다.
    EXPECT_TRUE(direction_enu.azimuth.isEquivalentTo(60_deg));
    EXPECT_EQ(direction_enu.elevation, 10_deg);
}

// ---------------------------------------------------------------- AntEnuTransform 검증

// 좌표 변환은 피치 한계의 양쪽 경계와 허용오차 안의 값을 거부한다.
TEST(AttitudeTransformTest, ThrowsOnGimbalLockPitch)
{
    // Given: 생성자에 전달할 입력을 준비한다.
    const beam::AttitudeConfig config(89_deg, math::SquareMatrix::identity(3), math::Tolerance{1e-6, 0.0});
    const beam::RadarAttitude rejected[] = {
        makeAttitude(0_deg, 89_deg, 0_deg), makeAttitude(0_deg, 88.9999999999_deg, 0_deg),
        makeAttitude(0_deg, -88.9999999999_deg, 0_deg), makeAttitude(0_deg, -90_deg, 0_deg)};
    const auto accepted = makeAttitude(0_deg, 88.9_deg, 0_deg);

    // When: 준비한 값으로 대상 객체 생성을 시도한다.
    const auto construct = [&](const beam::RadarAttitude& attitude) {
        return beam::AntEnuTransform(attitude, config);
    };

    // Then: 좌표 변환은 피치 한계의 양쪽 경계와 허용오차 안의 값을 거부한다.
    for (const auto& attitude : rejected)
    {
        EXPECT_THROW(construct(attitude), std::invalid_argument);
    }
    EXPECT_NO_THROW(construct(accepted));
}

// 좌표 변환은 장착 회전을 안테나 방향에 반영한다.
TEST(AttitudeTransformTest, MountRotationIsAppliedToAntDirection)
{
    // Given: 90도 장착 회전이 있는 좌표 변환을 준비한다.
    const beam::AttitudeConfig config = beam::AttitudeConfig::Builder()
        .maxAbsPitch(89_deg)
        .mountAntToBodyRotation(math::SquareMatrix::identity(3))
        .mountOrthogonalTolerance(math::Tolerance{1e-6, 0.0})
        .mountAntToBodyRotation(math::SquareMatrix{{0, -1, 0}, {1, 0, 0}, {0, 0, 1}})
        .build();
    const beam::AntEnuTransform transform(makeAttitude(0_deg, 0_deg, 0_deg), config);

    // 안테나 정면(방위각 0도)이 몸체 기준 -90도(서쪽)로 장착되어 있다.

    // When: 안테나 정면을 ENU 방향으로 변환한다.
    const auto direction_enu = transform.antToEnu(math::SphericalDirection(0_deg, 0_deg));

    // Then: 좌표 변환은 장착 회전을 안테나 방향에 반영한다.
    EXPECT_TRUE(direction_enu.azimuth.isEquivalentTo(-90_deg));
    EXPECT_EQ(direction_enu.elevation, 0_deg);
}

// 좌표 변환은 설정에서 받은 피치 한계로 짐벌락을 판정한다.
TEST(AttitudeTransformTest, MaxAbsPitchFromConfigIsUsedForGimbalLock)
{
    // Given: 생성자에 전달할 입력을 준비한다.
    const beam::AttitudeConfig config(60_deg, math::SquareMatrix::identity(3), math::Tolerance{1e-6, 0.0});
    const auto rejected = makeAttitude(0_deg, 60_deg, 0_deg);
    const auto accepted = makeAttitude(0_deg, 59.9_deg, 0_deg);

    // When: 준비한 값으로 대상 객체 생성을 시도한다.
    const auto construct = [&](const beam::RadarAttitude& attitude) {
        return beam::AntEnuTransform(attitude, config);
    };

    // Then: 좌표 변환은 설정에서 받은 피치 한계로 짐벌락을 판정한다.
    EXPECT_THROW(construct(rejected), std::invalid_argument);
    EXPECT_NO_THROW(construct(accepted));
}

// ---------------------------------------------------------------- 각도 변환

// 요 회전은 방위각을 시계 방향으로 회전시킨다.
TEST(AttitudeTransformTest, YawRotatesAzimuthClockwise)
{
    // Given: 요가 90도인 좌표 변환을 준비한다.
    const beam::AntEnuTransform transform = makeValidTransform(0_deg, 0_deg, 90_deg); // 정면이 동쪽

    // When: 안테나 정면을 ENU 방향으로 변환한다.
    const auto direction_enu = transform.antToEnu(math::SphericalDirection(0_deg, 0_deg));

    // Then: 요 회전은 방위각을 시계 방향으로 회전시킨다.
    EXPECT_TRUE(direction_enu.azimuth.isEquivalentTo(90_deg));
    EXPECT_EQ(direction_enu.elevation, 0_deg);
}

// 피치 회전은 안테나 정면의 고각을 올린다.
TEST(AttitudeTransformTest, PitchRaisesBoresight)
{
    // Given: 피치가 10도인 좌표 변환을 준비한다.
    const beam::AntEnuTransform transform = makeValidTransform(0_deg, 10_deg, 0_deg);

    // When: 안테나 정면을 ENU 방향으로 변환한다.
    const auto direction_enu = transform.antToEnu(math::SphericalDirection(0_deg, 0_deg));

    // Then: 피치 회전은 안테나 정면의 고각을 올린다.
    EXPECT_TRUE(direction_enu.azimuth.isEquivalentTo(0_deg));
    EXPECT_EQ(direction_enu.elevation, 10_deg);
}

// 정방향 변환 후 역방향 변환은 원래 방향을 복원한다.
TEST(AttitudeTransformTest, AntToEnuAndBackRestoresAngles)
{
    // Given: 세 축으로 회전한 좌표 변환을 준비한다.
    const beam::AntEnuTransform transform = makeValidTransform(10_deg, 20_deg, 30_deg);

    // When: 여러 방향을 ENU로 변환한 뒤 안테나 방향으로 되돌린다.
    for (math::Angle azimuth_ant : {-45.0_deg, -30.0_deg, 0.0_deg, 15.0_deg, 45.0_deg})
    {
        for (math::Angle elevation_ant : {0.0_deg, 10.0_deg, 41.2_deg})
        {
            const auto direction_enu = transform.antToEnu(math::SphericalDirection(azimuth_ant, elevation_ant));
            const auto directionRestored_ant = transform.enuToAnt(direction_enu);

            // Then: 정방향 변환 후 역방향 변환은 원래 방향을 복원한다.
            EXPECT_TRUE(directionRestored_ant.azimuth.isEquivalentTo(math::wrap360(azimuth_ant)));
            EXPECT_EQ(directionRestored_ant.elevation, elevation_ant);
        }
    }
}

// 좌표 변환은 후방 방위각을 180도로 반환한다.
TEST(AttitudeTransformTest, AzimuthUsesUnsignedRangeEndpoint)
{
    // Given: 영 자세의 좌표 변환을 준비한다.
    const beam::AntEnuTransform transform = makeValidTransform(0_deg, 0_deg, 0_deg);

    // When: 방위각 -180도인 안테나 방향을 변환한다.
    const auto directionAt180_enu = transform.antToEnu(math::SphericalDirection(-180_deg, 0_deg));

    // Then: 좌표 변환은 후방 방위각을 180도로 반환한다.
    EXPECT_DOUBLE_EQ(directionAt180_enu.azimuth.deg(), 180.0);
}
