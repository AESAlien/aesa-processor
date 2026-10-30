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
    return beam::RadarAttitude(roll, pitch, yaw);
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

TEST(AttitudeTransformTest, ZeroAttitudeKeepsAngles)
{
    const beam::AntEnuTransform transform = makeValidTransform(0_deg, 0_deg, 0_deg);

    const auto direction_enu = transform.antToEnu(math::SphericalDirection(math::CircularAngle(-30.0_deg), math::TiltAngle(10.0_deg)));

    EXPECT_DOUBLE_EQ(direction_enu.azimuth.deg(), 330.0);
    EXPECT_EQ(direction_enu.elevation, 10_deg);
}

TEST(AttitudeTransformTest, AcceptsAnglesCreatedFromRadians)
{
    const auto transform = makeValidTransform(0_deg, 0_deg, math::Angle::fromRadians(math::PI / 2.0));
    const auto direction_enu = transform.antToEnu(math::SphericalDirection(
        math::CircularAngle::fromRadians(-math::PI / 6.0),
        math::TiltAngle::fromRadians(math::PI / 18.0)));

    EXPECT_TRUE(direction_enu.azimuth.isEquivalentTo(60_deg));
    EXPECT_EQ(direction_enu.elevation, 10_deg);
}

// ---------------------------------------------------------------- RadarAttitude 검증

TEST(RadarAttitudeTest, StoresGivenValues)
{
    const beam::RadarAttitude attitude = makeAttitude(4_deg, 5_deg, 6_deg);

    EXPECT_TRUE(attitude.roll().isEquivalentTo(4_deg));
    EXPECT_EQ(attitude.pitch(), 5_deg);
    EXPECT_TRUE(attitude.yaw().isEquivalentTo(6_deg));
}

TEST(RadarAttitudeTest, ThrowsOnOutOfRangeValues)
{
    EXPECT_THROW(makeAttitude(181_deg, 0_deg, 0_deg), std::invalid_argument);
    EXPECT_THROW(makeAttitude(-181_deg, 0_deg, 0_deg), std::invalid_argument);
    EXPECT_THROW(makeAttitude(0_deg, 0_deg, 361_deg), std::invalid_argument);
    EXPECT_THROW(makeAttitude(0_deg, 0_deg, -361_deg), std::invalid_argument);
}

TEST(RadarAttitudeTest, ThrowsOnValuesEquivalentByPeriodToValidRange)
{
    // 360도 주기로는 범위 안과 동치이지만 실제 수치가 명시된 범위 밖이면 거부해야 한다.
    EXPECT_THROW(makeAttitude(540_deg, 0_deg, 0_deg), std::invalid_argument);
    EXPECT_THROW(makeAttitude(0_deg, 0_deg, 720_deg), std::invalid_argument);
    EXPECT_THROW(makeAttitude(0_deg, 0_deg, -720_deg), std::invalid_argument);
}

TEST(RadarAttitudeTest, AcceptsBoundaryAnglesIncludingEndpoints)
{
    EXPECT_NO_THROW(makeAttitude(180_deg, 0_deg, 360_deg));
    EXPECT_NO_THROW(makeAttitude(-180_deg, 0_deg, -360_deg));
}

TEST(RadarAttitudeTest, PitchRangeIsNotCheckedByRadarAttitude)
{
    // 짐벌락 판정은 AttitudeConfig에 의존하므로 AntEnuTransform의 책임이다.
    EXPECT_NO_THROW(makeAttitude(0_deg, 90_deg, 0_deg));
    EXPECT_NO_THROW(makeAttitude(0_deg, -95_deg, 0_deg));
}

// ---------------------------------------------------------------- AntEnuTransform 검증

TEST(AttitudeTransformTest, ThrowsOnGimbalLockPitch)
{
    const beam::AttitudeConfig config(
        89_deg,
        math::SquareMatrix::identity(3),
        math::Tolerance{1e-6, 0.0});

    EXPECT_THROW(beam::AntEnuTransform(makeAttitude(0_deg, 89_deg, 0_deg), config), std::invalid_argument);
    EXPECT_THROW(beam::AntEnuTransform(makeAttitude(0_deg, -95_deg, 0_deg), config), std::invalid_argument);
    EXPECT_NO_THROW(beam::AntEnuTransform(makeAttitude(0_deg, 88.9_deg, 0_deg), config));
}

TEST(AttitudeTransformTest, AcceptsRotationMount)
{
    const beam::AttitudeConfig config = beam::AttitudeConfig::Builder()
        .maxAbsPitch(89_deg)
        .mountAntToBodyRotation(math::SquareMatrix::identity(3))
        .mountOrthogonalTolerance(math::Tolerance{1e-6, 0.0})
        .mountAntToBodyRotation(math::SquareMatrix{{0, -1, 0}, {1, 0, 0}, {0, 0, 1}}) // z축 90도 회전
        .build();

    EXPECT_NO_THROW(beam::AntEnuTransform(makeAttitude(0_deg, 0_deg, 0_deg), config));
}

TEST(AttitudeTransformTest, MountRotationIsAppliedToAntDirection)
{
    const beam::AttitudeConfig config = beam::AttitudeConfig::Builder()
        .maxAbsPitch(89_deg)
        .mountAntToBodyRotation(math::SquareMatrix::identity(3))
        .mountOrthogonalTolerance(math::Tolerance{1e-6, 0.0})
        .mountAntToBodyRotation(math::SquareMatrix{{0, -1, 0}, {1, 0, 0}, {0, 0, 1}})
        .build();
    const beam::AntEnuTransform transform(makeAttitude(0_deg, 0_deg, 0_deg), config);

    // 안테나 정면(방위각 0도)이 몸체 기준 -90도(서쪽)로 장착되어 있다.
    const auto direction_enu = transform.antToEnu(math::SphericalDirection(math::CircularAngle(0_deg), math::TiltAngle(0_deg)));
    EXPECT_TRUE(direction_enu.azimuth.isEquivalentTo(-90_deg));
    EXPECT_EQ(direction_enu.elevation, 0_deg);
}

TEST(AttitudeTransformTest, MaxAbsPitchFromConfigIsUsedForGimbalLock)
{
    const beam::AttitudeConfig config = beam::AttitudeConfig::Builder()
        .maxAbsPitch(60_deg)
        .mountAntToBodyRotation(math::SquareMatrix::identity(3))
        .mountOrthogonalTolerance(math::Tolerance{1e-6, 0.0})
        .build();

    EXPECT_THROW(beam::AntEnuTransform(makeAttitude(0_deg, 60_deg, 0_deg), config), std::invalid_argument);
    EXPECT_NO_THROW(beam::AntEnuTransform(makeAttitude(0_deg, 59.9_deg, 0_deg), config));
}

// ---------------------------------------------------------------- 각도 변환

TEST(AttitudeTransformTest, IdentityKeepsAngles)
{
    const beam::AntEnuTransform transform = makeValidTransform(0_deg, 0_deg, 0_deg);
    const auto direction_enu = transform.antToEnu(math::SphericalDirection(math::CircularAngle(-30_deg), math::TiltAngle(10_deg)));
    EXPECT_DOUBLE_EQ(direction_enu.azimuth.deg(), 330.0);
    EXPECT_EQ(direction_enu.elevation, 10_deg);
}

TEST(AttitudeTransformTest, YawRotatesAzimuthClockwise)
{
    const beam::AntEnuTransform transform = makeValidTransform(0_deg, 0_deg, 90_deg); // 정면이 동쪽
    const auto direction_enu = transform.antToEnu(math::SphericalDirection(math::CircularAngle(0_deg), math::TiltAngle(0_deg)));
    EXPECT_TRUE(direction_enu.azimuth.isEquivalentTo(90_deg));
    EXPECT_EQ(direction_enu.elevation, 0_deg);
}

TEST(AttitudeTransformTest, PitchRaisesBoresight)
{
    const beam::AntEnuTransform transform = makeValidTransform(0_deg, 10_deg, 0_deg);
    const auto direction_enu = transform.antToEnu(math::SphericalDirection(math::CircularAngle(0_deg), math::TiltAngle(0_deg)));
    EXPECT_TRUE(direction_enu.azimuth.isEquivalentTo(0_deg));
    EXPECT_EQ(direction_enu.elevation, 10_deg);
}

TEST(AttitudeTransformTest, AntToEnuAndBackRestoresAngles)
{
    const beam::AntEnuTransform transform = makeValidTransform(10_deg, 20_deg, 30_deg);

    for (math::Angle azimuth_ant : {-45.0_deg, -30.0_deg, 0.0_deg, 15.0_deg, 45.0_deg})
    {
        for (math::Angle elevation_ant : {0.0_deg, 10.0_deg, 41.2_deg})
        {
            const auto direction_enu = transform.antToEnu(math::SphericalDirection(math::CircularAngle(azimuth_ant), math::TiltAngle(elevation_ant)));
            const auto directionRestored_ant = transform.enuToAnt(direction_enu);
            EXPECT_TRUE(directionRestored_ant.azimuth.isEquivalentTo(math::wrap360(azimuth_ant)));
            EXPECT_EQ(directionRestored_ant.elevation, elevation_ant);
        }
    }
}

TEST(AttitudeTransformTest, AzimuthUsesUnsignedRangeEndpoint)
{
    const beam::AntEnuTransform transform = makeValidTransform(0_deg, 0_deg, 0_deg);
    const auto directionAt180_enu = transform.antToEnu(math::SphericalDirection(math::CircularAngle(-180_deg), math::TiltAngle(0_deg)));
    EXPECT_DOUBLE_EQ(directionAt180_enu.azimuth.deg(), 180.0);
}
