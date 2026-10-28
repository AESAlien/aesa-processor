#include "attitude/ant_enu_transform.hpp"
#include "attitude/attitude_error.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <math/trigonometry.hpp>

using namespace math::literals;

namespace
{

beam::RadarAttitude makeAttitude(
    math::Angle latitude,
    math::Angle longitude,
    math::Length altitude,
    math::Angle roll,
    math::Angle pitch,
    math::Angle yaw
) {
    return beam::RadarAttitude(latitude, longitude, altitude, roll, pitch, yaw);
}

beam::RadarAttitude makeAttitude(math::Angle roll, math::Angle pitch, math::Angle yaw)
{
    return makeAttitude(37.0_deg, 127.0_deg, 0.1_km, roll, pitch, yaw);
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

// 실패 시 던지는 AttitudeError 의 code 를 확인하는 헬퍼
#define EXPECT_ATTITUDE_ERROR(expression, expectedCode)                                                                \
    do                                                                                                                 \
    {                                                                                                                  \
        try                                                                                                            \
        {                                                                                                              \
            (void)(expression);                                                                                        \
            ADD_FAILURE() << "AttitudeError not thrown";                                                               \
        }                                                                                                              \
        catch (const beam::AttitudeError& error)                                                                       \
        {                                                                                                              \
            EXPECT_EQ(error.code(), expectedCode);                                                                     \
        }                                                                                                              \
    } while (0)

// ---------------------------------------------------------------- makeAntToEnu

TEST(AttitudeTransformTest, ZeroAttitudeKeepsAngles)
{
    const beam::AntEnuTransform transform = makeValidTransform(0_deg, 0_deg, 0_deg);

    const auto direction_enu = transform.antToEnu(math::SphericalDirection(math::Azimuth(-30.0_deg), math::Elevation(10.0_deg)));

    EXPECT_DOUBLE_EQ(direction_enu.azimuth.deg(), 330.0);
    EXPECT_EQ(direction_enu.elevation, 10_deg);
}

TEST(AttitudeTransformTest, AcceptsAnglesCreatedFromRadians)
{
    const auto transform = makeValidTransform(0_deg, 0_deg, math::Angle::fromRadians(math::PI / 2.0));
    const auto direction_enu = transform.antToEnu(math::SphericalDirection(
        math::Azimuth::fromRadians(-math::PI / 6.0),
        math::Elevation::fromRadians(math::PI / 18.0)));

    EXPECT_TRUE(direction_enu.azimuth.isEquivalentTo(60_deg));
    EXPECT_EQ(direction_enu.elevation, 10_deg);
}

// ---------------------------------------------------------------- RadarAttitude 검증

TEST(RadarAttitudeTest, StoresGivenValues)
{
    const beam::RadarAttitude attitude = makeAttitude(1_deg, 2_deg, 3_km, 4_deg, 5_deg, 6_deg);

    EXPECT_EQ(attitude.latitude(), 1_deg);
    EXPECT_TRUE(attitude.longitude().isEquivalentTo(2_deg));
    EXPECT_EQ(attitude.altitude(), 3_km);
    EXPECT_TRUE(attitude.roll().isEquivalentTo(4_deg));
    EXPECT_EQ(attitude.pitch(), 5_deg);
    EXPECT_TRUE(attitude.yaw().isEquivalentTo(6_deg));
}

TEST(RadarAttitudeTest, ThrowsOnOutOfRangeValues)
{
    EXPECT_ATTITUDE_ERROR(makeAttitude(91_deg, 127_deg, 0.1_km, 0_deg, 0_deg, 0_deg), beam::AttitudeErrorCode::OUT_OF_RANGE);
    EXPECT_ATTITUDE_ERROR(makeAttitude(37_deg, -181_deg, 0.1_km, 0_deg, 0_deg, 0_deg), beam::AttitudeErrorCode::OUT_OF_RANGE);
    EXPECT_ATTITUDE_ERROR(makeAttitude(37_deg, 127_deg, 101_km, 0_deg, 0_deg, 0_deg), beam::AttitudeErrorCode::OUT_OF_RANGE);
    EXPECT_ATTITUDE_ERROR(makeAttitude(181_deg, 0_deg, 0_deg), beam::AttitudeErrorCode::OUT_OF_RANGE);
    EXPECT_ATTITUDE_ERROR(makeAttitude(0_deg, 0_deg, 361_deg), beam::AttitudeErrorCode::OUT_OF_RANGE);
}

TEST(RadarAttitudeTest, ThrowsOnValuesEquivalentByPeriodToValidRange)
{
    // 360도 주기로는 범위 안과 동치이지만 실제 수치가 명시된 범위 밖이면 거부해야 한다.
    EXPECT_ATTITUDE_ERROR(makeAttitude(450_deg, 127_deg, 0.1_km, 0_deg, 0_deg, 0_deg), beam::AttitudeErrorCode::OUT_OF_RANGE);
    EXPECT_ATTITUDE_ERROR(makeAttitude(-270_deg, 127_deg, 0.1_km, 0_deg, 0_deg, 0_deg), beam::AttitudeErrorCode::OUT_OF_RANGE);
    EXPECT_ATTITUDE_ERROR(makeAttitude(37_deg, 540_deg, 0.1_km, 0_deg, 0_deg, 0_deg), beam::AttitudeErrorCode::OUT_OF_RANGE);
    EXPECT_ATTITUDE_ERROR(makeAttitude(37_deg, -540_deg, 0.1_km, 0_deg, 0_deg, 0_deg), beam::AttitudeErrorCode::OUT_OF_RANGE);
    EXPECT_ATTITUDE_ERROR(makeAttitude(540_deg, 0_deg, 0_deg), beam::AttitudeErrorCode::OUT_OF_RANGE);
    EXPECT_ATTITUDE_ERROR(makeAttitude(0_deg, 0_deg, 720_deg), beam::AttitudeErrorCode::OUT_OF_RANGE);
    EXPECT_ATTITUDE_ERROR(makeAttitude(0_deg, 0_deg, -720_deg), beam::AttitudeErrorCode::OUT_OF_RANGE);
}

TEST(RadarAttitudeTest, ValidatesAltitudeBoundariesWithLengthLiterals)
{
    EXPECT_NO_THROW(makeAttitude(37_deg, 127_deg, -0.5_km, 0_deg, 0_deg, 0_deg));
    EXPECT_NO_THROW(makeAttitude(37_deg, 127_deg, 100_km, 0_deg, 0_deg, 0_deg));

    EXPECT_ATTITUDE_ERROR(makeAttitude(37_deg, 127_deg, -0.501_km, 0_deg, 0_deg, 0_deg), beam::AttitudeErrorCode::OUT_OF_RANGE);
    EXPECT_ATTITUDE_ERROR(makeAttitude(37_deg, 127_deg, 100.001_km, 0_deg, 0_deg, 0_deg), beam::AttitudeErrorCode::OUT_OF_RANGE);
}

TEST(RadarAttitudeTest, AcceptsBoundaryAnglesIncludingEndpoints)
{
    EXPECT_NO_THROW(makeAttitude(90_deg, 180_deg, 0_km, 180_deg, 0_deg, 360_deg));
    EXPECT_NO_THROW(makeAttitude(-90_deg, -180_deg, 0_km, -180_deg, 0_deg, -360_deg));
}

TEST(RadarAttitudeTest, PitchRangeIsNotCheckedByRadarAttitude)
{
    // 짐벌락 판정은 AttitudeConfig에 의존하므로 AntEnuTransform의 책임이다.
    EXPECT_NO_THROW(makeAttitude(0_deg, 90_deg, 0_deg));
    EXPECT_NO_THROW(makeAttitude(0_deg, -95_deg, 0_deg));
}

TEST(RadarAttitudeTest, AttitudeErrorIsInvalidArgument)
{
    EXPECT_THROW(makeAttitude(91_deg, 127_deg, 0.1_km, 0_deg, 0_deg, 0_deg), std::invalid_argument);
}

// ---------------------------------------------------------------- AntEnuTransform 검증

TEST(AttitudeTransformTest, ThrowsOnGimbalLockPitch)
{
    const beam::AttitudeConfig config(
        89_deg,
        math::SquareMatrix::identity(3),
        math::Tolerance{1e-6, 0.0});

    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(makeAttitude(0_deg, 89_deg, 0_deg), config), beam::AttitudeErrorCode::GIMBAL_LOCK);
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(makeAttitude(0_deg, -95_deg, 0_deg), config), beam::AttitudeErrorCode::GIMBAL_LOCK);
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
    const auto direction_enu = transform.antToEnu(math::SphericalDirection(math::Azimuth(0_deg), math::Elevation(0_deg)));
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

    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(makeAttitude(0_deg, 60_deg, 0_deg), config), beam::AttitudeErrorCode::GIMBAL_LOCK);
    EXPECT_NO_THROW(beam::AntEnuTransform(makeAttitude(0_deg, 59.9_deg, 0_deg), config));
}

TEST(AttitudeTransformTest, AttitudeErrorIsInvalidArgument)
{
    EXPECT_THROW(
        beam::AntEnuTransform(
            makeAttitude(0_deg, 95_deg, 0_deg),
            beam::AttitudeConfig(
                89_deg,
                math::SquareMatrix::identity(3),
                math::Tolerance{1e-6, 0.0})),
        std::invalid_argument);
}

// ---------------------------------------------------------------- 각도 변환

TEST(AttitudeTransformTest, IdentityKeepsAngles)
{
    const beam::AntEnuTransform transform = makeValidTransform(0_deg, 0_deg, 0_deg);
    const auto direction_enu = transform.antToEnu(math::SphericalDirection(math::Azimuth(-30_deg), math::Elevation(10_deg)));
    EXPECT_DOUBLE_EQ(direction_enu.azimuth.deg(), 330.0);
    EXPECT_EQ(direction_enu.elevation, 10_deg);
}

TEST(AttitudeTransformTest, YawRotatesAzimuthClockwise)
{
    const beam::AntEnuTransform transform = makeValidTransform(0_deg, 0_deg, 90_deg); // 정면이 동쪽
    const auto direction_enu = transform.antToEnu(math::SphericalDirection(math::Azimuth(0_deg), math::Elevation(0_deg)));
    EXPECT_TRUE(direction_enu.azimuth.isEquivalentTo(90_deg));
    EXPECT_EQ(direction_enu.elevation, 0_deg);
}

TEST(AttitudeTransformTest, PitchRaisesBoresight)
{
    const beam::AntEnuTransform transform = makeValidTransform(0_deg, 10_deg, 0_deg);
    const auto direction_enu = transform.antToEnu(math::SphericalDirection(math::Azimuth(0_deg), math::Elevation(0_deg)));
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
            const auto direction_enu = transform.antToEnu(math::SphericalDirection(math::Azimuth(azimuth_ant), math::Elevation(elevation_ant)));
            const auto directionRestored_ant = transform.enuToAnt(direction_enu);
            EXPECT_TRUE(directionRestored_ant.azimuth.isEquivalentTo(math::wrap360(azimuth_ant)));
            EXPECT_EQ(directionRestored_ant.elevation, elevation_ant);
        }
    }
}

TEST(AttitudeTransformTest, AzimuthUsesUnsignedRangeEndpoint)
{
    const beam::AntEnuTransform transform = makeValidTransform(0_deg, 0_deg, 0_deg);
    const auto directionAt180_enu = transform.antToEnu(math::SphericalDirection(math::Azimuth(-180_deg), math::Elevation(0_deg)));
    EXPECT_DOUBLE_EQ(directionAt180_enu.azimuth.deg(), 180.0);
}
