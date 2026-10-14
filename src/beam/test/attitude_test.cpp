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
    math::Distance altitude,
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
    return beam::AntEnuTransform(makeAttitude(roll, pitch, yaw), beam::AttitudeConfig{});
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

    const auto direction_enu = transform.antToEnu(math::SphericalDirection(-30.0_deg, 10.0_deg));

    EXPECT_EQ(direction_enu.azimuth(), -30_deg);
    EXPECT_EQ(direction_enu.elevation(), 10_deg);
}

TEST(AttitudeTransformTest, AcceptsAnglesCreatedFromRadians)
{
    const auto transform = makeValidTransform(0_deg, 0_deg, math::Angle::fromRadians(math::PI / 2.0));
    const auto direction_enu = transform.antToEnu(math::SphericalDirection(
        math::Angle::fromRadians(-math::PI / 6.0), math::Angle::fromRadians(math::PI / 18.0)));

    EXPECT_EQ(direction_enu.azimuth(), 60_deg);
    EXPECT_EQ(direction_enu.elevation(), 10_deg);
}

// ---------------------------------------------------------------- RadarAttitude 검증

TEST(RadarAttitudeTest, StoresGivenValues)
{
    const beam::RadarAttitude attitude = makeAttitude(1_deg, 2_deg, 3_km, 4_deg, 5_deg, 6_deg);

    EXPECT_EQ(attitude.latitude(), 1_deg);
    EXPECT_EQ(attitude.longitude(), 2_deg);
    EXPECT_EQ(attitude.altitude(), 3_km);
    EXPECT_EQ(attitude.roll(), 4_deg);
    EXPECT_EQ(attitude.pitch(), 5_deg);
    EXPECT_EQ(attitude.yaw(), 6_deg);
}

TEST(RadarAttitudeTest, ThrowsOnNonFiniteValues)
{
    const auto nan = math::Angle::fromDegrees(std::numeric_limits<double>::quiet_NaN());
    const auto inf = math::Distance::fromKilometers(std::numeric_limits<double>::infinity());

    EXPECT_ATTITUDE_ERROR(makeAttitude(nan, 0_deg, 0_km, 0_deg, 0_deg, 0_deg), beam::AttitudeErrorCode::NOT_FINITE);
    EXPECT_ATTITUDE_ERROR(makeAttitude(0_deg, nan, 0_km, 0_deg, 0_deg, 0_deg), beam::AttitudeErrorCode::NOT_FINITE);
    EXPECT_ATTITUDE_ERROR(makeAttitude(0_deg, 0_deg, inf, 0_deg, 0_deg, 0_deg), beam::AttitudeErrorCode::NOT_FINITE);
    EXPECT_ATTITUDE_ERROR(makeAttitude(0_deg, 0_deg, 0_km, nan, 0_deg, 0_deg), beam::AttitudeErrorCode::NOT_FINITE);
    EXPECT_ATTITUDE_ERROR(makeAttitude(0_deg, 0_deg, 0_km, 0_deg, nan, 0_deg), beam::AttitudeErrorCode::NOT_FINITE);
    EXPECT_ATTITUDE_ERROR(makeAttitude(0_deg, 0_deg, 0_km, 0_deg, 0_deg, nan), beam::AttitudeErrorCode::NOT_FINITE);
}

TEST(RadarAttitudeTest, NonFiniteIsReportedBeforeOutOfRange)
{
    const auto nan = math::Angle::fromDegrees(std::numeric_limits<double>::quiet_NaN());

    // 위도는 범위를 벗어났지만 roll이 NaN이므로 NOT_FINITE가 먼저 보고된다.
    EXPECT_ATTITUDE_ERROR(makeAttitude(91_deg, 0_deg, 0_km, nan, 0_deg, 0_deg), beam::AttitudeErrorCode::NOT_FINITE);
}

TEST(RadarAttitudeTest, ThrowsOnOutOfRangeValues)
{
    EXPECT_ATTITUDE_ERROR(makeAttitude(91_deg, 127_deg, 0.1_km, 0_deg, 0_deg, 0_deg), beam::AttitudeErrorCode::OUT_OF_RANGE);
    EXPECT_ATTITUDE_ERROR(makeAttitude(37_deg, -181_deg, 0.1_km, 0_deg, 0_deg, 0_deg), beam::AttitudeErrorCode::OUT_OF_RANGE);
    EXPECT_ATTITUDE_ERROR(makeAttitude(37_deg, 127_deg, 101_km, 0_deg, 0_deg, 0_deg), beam::AttitudeErrorCode::OUT_OF_RANGE);
    EXPECT_ATTITUDE_ERROR(makeAttitude(181_deg, 0_deg, 0_deg), beam::AttitudeErrorCode::OUT_OF_RANGE);
    EXPECT_ATTITUDE_ERROR(makeAttitude(0_deg, 0_deg, 361_deg), beam::AttitudeErrorCode::OUT_OF_RANGE);
}

TEST(RadarAttitudeTest, ValidatesAltitudeBoundariesWithDistanceLiterals)
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

TEST(RadarAttitudeTest, PitchIsOnlyCheckedForFiniteness)
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
    const beam::AttitudeConfig config;

    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(makeAttitude(0_deg, 89_deg, 0_deg), config), beam::AttitudeErrorCode::GIMBAL_LOCK);
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(makeAttitude(0_deg, -95_deg, 0_deg), config), beam::AttitudeErrorCode::GIMBAL_LOCK);
    EXPECT_NO_THROW(beam::AntEnuTransform(makeAttitude(0_deg, 88.9_deg, 0_deg), config));
}

TEST(AttitudeTransformTest, AcceptsRotationMount)
{
    const beam::AttitudeConfig config = beam::AttitudeConfig::Builder()
        .mountAntToBodyRotation(math::SquareMatrix{{0, -1, 0}, {1, 0, 0}, {0, 0, 1}}) // z축 90도 회전
        .build();

    EXPECT_NO_THROW(beam::AntEnuTransform(makeAttitude(0_deg, 0_deg, 0_deg), config));
}

TEST(AttitudeTransformTest, MountRotationIsAppliedToAntDirection)
{
    const beam::AttitudeConfig config = beam::AttitudeConfig::Builder()
        .mountAntToBodyRotation(math::SquareMatrix{{0, -1, 0}, {1, 0, 0}, {0, 0, 1}})
        .build();
    const beam::AntEnuTransform transform(makeAttitude(0_deg, 0_deg, 0_deg), config);

    // 안테나 정면(방위각 0도)이 몸체 기준 -90도(서쪽)로 장착되어 있다.
    const auto direction_enu = transform.antToEnu(math::SphericalDirection(0_deg, 0_deg));
    EXPECT_EQ(direction_enu.azimuth(), -90_deg);
    EXPECT_EQ(direction_enu.elevation(), 0_deg);
}

TEST(AttitudeTransformTest, MaxAbsPitchFromConfigIsUsedForGimbalLock)
{
    const beam::AttitudeConfig config = beam::AttitudeConfig::Builder().maxAbsPitch(60_deg).build();

    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(makeAttitude(0_deg, 60_deg, 0_deg), config), beam::AttitudeErrorCode::GIMBAL_LOCK);
    EXPECT_NO_THROW(beam::AntEnuTransform(makeAttitude(0_deg, 59.9_deg, 0_deg), config));
}

TEST(AttitudeTransformTest, AttitudeErrorIsInvalidArgument)
{
    EXPECT_THROW(beam::AntEnuTransform(makeAttitude(0_deg, 95_deg, 0_deg), beam::AttitudeConfig{}), std::invalid_argument);
}

// ---------------------------------------------------------------- 각도 변환

TEST(AttitudeTransformTest, IdentityKeepsAngles)
{
    const beam::AntEnuTransform transform = makeValidTransform(0_deg, 0_deg, 0_deg);
    const auto direction_enu = transform.antToEnu(math::SphericalDirection(-30_deg, 10_deg));
    EXPECT_EQ(direction_enu.azimuth(), -30_deg); // 음수 방위각이 0~360으로 바뀌지 않아야 함
    EXPECT_EQ(direction_enu.elevation(), 10_deg);
}

TEST(AttitudeTransformTest, YawRotatesAzimuthClockwise)
{
    const beam::AntEnuTransform transform = makeValidTransform(0_deg, 0_deg, 90_deg); // 정면이 동쪽
    const auto direction_enu = transform.antToEnu(math::SphericalDirection(0_deg, 0_deg));
    EXPECT_EQ(direction_enu.azimuth(), 90_deg);
    EXPECT_EQ(direction_enu.elevation(), 0_deg);
}

TEST(AttitudeTransformTest, PitchRaisesBoresight)
{
    const beam::AntEnuTransform transform = makeValidTransform(0_deg, 10_deg, 0_deg);
    const auto direction_enu = transform.antToEnu(math::SphericalDirection(0_deg, 0_deg));
    EXPECT_EQ(direction_enu.azimuth(), 0_deg);
    EXPECT_EQ(direction_enu.elevation(), 10_deg);
}

TEST(AttitudeTransformTest, AntToEnuAndBackRestoresAngles)
{
    const beam::AntEnuTransform transform = makeValidTransform(10_deg, 20_deg, 30_deg);

    for (math::Angle azimuth_ant : {-45.0_deg, -30.0_deg, 0.0_deg, 15.0_deg, 45.0_deg})
    {
        for (math::Angle elevation_ant : {0.0_deg, 10.0_deg, 41.2_deg})
        {
            const auto direction_enu = transform.antToEnu(math::SphericalDirection(azimuth_ant, elevation_ant));
            const auto directionRestored_ant = transform.enuToAnt(direction_enu);
            EXPECT_EQ(directionRestored_ant.azimuth(), azimuth_ant);
            EXPECT_EQ(directionRestored_ant.elevation(), elevation_ant);
        }
    }
}

TEST(AttitudeTransformTest, AzimuthKeepsSignedRangeEndpoints)
{
    const beam::AntEnuTransform transform = makeValidTransform(0_deg, 0_deg, 0_deg);
    const auto directionAt180_enu = transform.antToEnu(math::SphericalDirection(-180_deg, 0_deg));
    EXPECT_EQ(math::abs(directionAt180_enu.azimuth()), 180_deg);
}
