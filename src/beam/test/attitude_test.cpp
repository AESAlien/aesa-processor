#include "attitude/ant_enu_transform.hpp"
#include "attitude/attitude_error.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{

constexpr double TOLERANCE = 1e-9;

beam::RadarAttitude makeAttitude(double roll_deg, double pitch_deg, double yaw_deg)
{
    beam::RadarAttitude attitude{};
    attitude.latitude_deg = 37.0;
    attitude.longitude_deg = 127.0;
    attitude.altitude_km = 0.1;
    attitude.roll_deg = roll_deg;
    attitude.pitch_deg = pitch_deg;
    attitude.yaw_deg = yaw_deg;
    return attitude;
}

beam::AntEnuTransform makeValidTransform(double roll_deg, double pitch_deg, double yaw_deg)
{
    return beam::AntEnuTransform(makeAttitude(roll_deg, pitch_deg, yaw_deg), beam::AttitudeConfig{});
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
    const beam::AntEnuTransform transform = makeValidTransform(0, 0, 0);

    const auto [azimuth_enu_deg, elevation_enu_deg] = transform.antToEnu(-30.0, 10.0);

    EXPECT_NEAR(azimuth_enu_deg, -30.0, TOLERANCE);
    EXPECT_NEAR(elevation_enu_deg, 10.0, TOLERANCE);
}

TEST(AttitudeTransformTest, ThrowsOnNonFiniteValues)
{
    const beam::AttitudeConfig config;
    beam::RadarAttitude attitude = makeAttitude(0, 0, 0);

    attitude.roll_deg = std::numeric_limits<double>::quiet_NaN();
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(attitude, config), beam::AttitudeErrorCode::NOT_FINITE);

    attitude = makeAttitude(0, 0, 0);
    attitude.altitude_km = std::numeric_limits<double>::infinity();
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(attitude, config), beam::AttitudeErrorCode::NOT_FINITE);
}

TEST(AttitudeTransformTest, ThrowsOnOutOfRangeValues)
{
    const beam::AttitudeConfig config;
    beam::RadarAttitude attitude;

    attitude = makeAttitude(0, 0, 0);
    attitude.latitude_deg = 91;
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(attitude, config), beam::AttitudeErrorCode::OUT_OF_RANGE);

    attitude = makeAttitude(0, 0, 0);
    attitude.longitude_deg = -181;
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(attitude, config), beam::AttitudeErrorCode::OUT_OF_RANGE);

    attitude = makeAttitude(0, 0, 0);
    attitude.altitude_km = 101;
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(attitude, config), beam::AttitudeErrorCode::OUT_OF_RANGE);

    attitude = makeAttitude(181, 0, 0);
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(attitude, config), beam::AttitudeErrorCode::OUT_OF_RANGE);

    attitude = makeAttitude(0, 0, 361);
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(attitude, config), beam::AttitudeErrorCode::OUT_OF_RANGE);
}

TEST(AttitudeTransformTest, ThrowsOnGimbalLockPitch)
{
    const beam::AttitudeConfig config;

    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(makeAttitude(0, 89, 0), config), beam::AttitudeErrorCode::GIMBAL_LOCK);
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(makeAttitude(0, -95, 0), config), beam::AttitudeErrorCode::GIMBAL_LOCK);
    EXPECT_NO_THROW(beam::AntEnuTransform(makeAttitude(0, 88.9, 0), config));
}

TEST(AttitudeTransformTest, AcceptsRotationMount)
{
    beam::AttitudeConfig config;
    config.mountAntToBodyRotation = math::Matrix{{0, -1, 0}, {1, 0, 0}, {0, 0, 1}}; // z축 90도 회전

    EXPECT_NO_THROW(beam::AntEnuTransform(makeAttitude(0, 0, 0), config));
}

TEST(AttitudeTransformTest, ThrowsOnInvalidMount)
{
    beam::AttitudeConfig config;
    const auto attitude = makeAttitude(0, 0, 0);

    config.mountAntToBodyRotation = math::Matrix{{2, 0, 0}, {0, 1, 0}, {0, 0, 1}}; // 스케일
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(attitude, config), beam::AttitudeErrorCode::INVALID_MOUNT);

    config.mountAntToBodyRotation = math::Matrix{{1, 0, 0}, {0, 1, 0}, {0, 0, -1}}; // 반사 (det = -1)
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(attitude, config), beam::AttitudeErrorCode::INVALID_MOUNT);

    config.mountAntToBodyRotation = math::Matrix{{1, 0, 0}, {0, 1, 0}, {0, 0, 0}}; // 특이 행렬
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(attitude, config), beam::AttitudeErrorCode::INVALID_MOUNT);

    config.mountAntToBodyRotation = math::Matrix{{1, 0, 0}, {0, 1, 0}, {0, 0, std::numeric_limits<double>::infinity()}};
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(attitude, config), beam::AttitudeErrorCode::INVALID_MOUNT);

    config.mountAntToBodyRotation = math::Matrix::identity(2); // 3x3 이 아님
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(attitude, config), beam::AttitudeErrorCode::INVALID_MOUNT);
}

TEST(AttitudeTransformTest, AttitudeErrorIsInvalidArgument)
{
    EXPECT_THROW(beam::AntEnuTransform(makeAttitude(0, 95, 0), beam::AttitudeConfig{}), std::invalid_argument);
}

// ---------------------------------------------------------------- 각도 변환

TEST(AttitudeTransformTest, IdentityKeepsAngles)
{
    const beam::AntEnuTransform transform = makeValidTransform(0, 0, 0);
    const auto [azimuth_enu_deg, elevation_enu_deg] = transform.antToEnu(-30, 10);
    EXPECT_NEAR(azimuth_enu_deg, -30, 1e-9); // 음수 방위각이 0~360으로 바뀌지 않아야 함
    EXPECT_NEAR(elevation_enu_deg, 10, 1e-9);
}

TEST(AttitudeTransformTest, YawRotatesAzimuthClockwise)
{
    const beam::AntEnuTransform transform = makeValidTransform(0, 0, 90); // 정면이 동쪽
    const auto [azimuth_enu_deg, elevation_enu_deg] = transform.antToEnu(0, 0);
    EXPECT_NEAR(azimuth_enu_deg, 90, 1e-9);
    EXPECT_NEAR(elevation_enu_deg, 0, 1e-9);
}

TEST(AttitudeTransformTest, PitchRaisesBoresight)
{
    const beam::AntEnuTransform transform = makeValidTransform(0, 10, 0);
    const auto [azimuth_enu_deg, elevation_enu_deg] = transform.antToEnu(0, 0);
    EXPECT_NEAR(azimuth_enu_deg, 0, 1e-9);
    EXPECT_NEAR(elevation_enu_deg, 10, 1e-9);
}

TEST(AttitudeTransformTest, AntToEnuAndBackRestoresAngles)
{
    const beam::AntEnuTransform transform = makeValidTransform(10, 20, 30);

    for (double azimuth_ant_deg : {-45.0, -30.0, 0.0, 15.0, 45.0})
    {
        for (double elevation_ant_deg : {0.0, 10.0, 41.2})
        {
            const auto [azimuth_enu_deg, elevation_enu_deg] = transform.antToEnu(azimuth_ant_deg, elevation_ant_deg);
            const auto [azimuthRestored_ant_deg, elevationRestored_ant_deg] =
                transform.enuToAnt(azimuth_enu_deg, elevation_enu_deg);
            EXPECT_NEAR(azimuthRestored_ant_deg, azimuth_ant_deg, 1e-9);
            EXPECT_NEAR(elevationRestored_ant_deg, elevation_ant_deg, 1e-9);
        }
    }
}

TEST(AttitudeTransformTest, AzimuthIsWrappedToSignedRange)
{
    const beam::AntEnuTransform transform = makeValidTransform(0, 0, 0);
    const auto [azimuthAbove180_enu_deg, elevationAbove180_enu_deg] = transform.antToEnu(181, 0);
    EXPECT_NEAR(azimuthAbove180_enu_deg, -179, 1e-9);

    const auto [azimuthBelow180_enu_deg, elevationBelow180_enu_deg] = transform.antToEnu(-181, 0);
    EXPECT_NEAR(azimuthBelow180_enu_deg, 179, 1e-9);

    const auto [azimuthAt180_enu_deg, elevationAt180_enu_deg] = transform.antToEnu(180, 0);
    EXPECT_NEAR(std::fabs(azimuthAt180_enu_deg), 180, 1e-9); // 후방은 ±180 (0이 되면 안 됨)
}
