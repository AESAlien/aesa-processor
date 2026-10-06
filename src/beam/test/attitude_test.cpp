#include "attitude/ant_enu_transform.hpp"
#include "attitude/attitude_error.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

using namespace math::literals;

namespace
{

constexpr double TOLERANCE = 1e-9;

beam::RadarAttitude makeAttitude(math::Angle roll, math::Angle pitch, math::Angle yaw)
{
    beam::RadarAttitude attitude{};
    attitude.latitude = 37.0_deg;
    attitude.longitude = 127.0_deg;
    attitude.altitude = 0.1_km;
    attitude.roll = roll;
    attitude.pitch = pitch;
    attitude.yaw = yaw;
    return attitude;
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

    const auto [azimuth_enu, elevation_enu] = transform.antToEnu(-30.0_deg, 10.0_deg);

    EXPECT_NEAR(azimuth_enu.deg(), -30.0, TOLERANCE);
    EXPECT_NEAR(elevation_enu.deg(), 10.0, TOLERANCE);
}

TEST(AttitudeTransformTest, AcceptsAnglesCreatedFromRadians)
{
    const auto transform = makeValidTransform(0_deg, 0_deg, math::Angle::fromRadians(math::PI / 2.0));
    const auto [azimuth_enu, elevation_enu] = transform.antToEnu(
        math::Angle::fromRadians(-math::PI / 6.0), math::Angle::fromRadians(math::PI / 18.0));

    EXPECT_NEAR(azimuth_enu.deg(), 60.0, TOLERANCE);
    EXPECT_NEAR(elevation_enu.deg(), 10.0, TOLERANCE);
}

TEST(AttitudeTransformTest, ThrowsOnNonFiniteValues)
{
    const beam::AttitudeConfig config;
    beam::RadarAttitude attitude = makeAttitude(0_deg, 0_deg, 0_deg);

    attitude.roll = math::Angle::fromDegrees(std::numeric_limits<double>::quiet_NaN());
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(attitude, config), beam::AttitudeErrorCode::NOT_FINITE);

    attitude = makeAttitude(0_deg, 0_deg, 0_deg);
    attitude.altitude = math::Distance::fromKilometers(std::numeric_limits<double>::infinity());
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(attitude, config), beam::AttitudeErrorCode::NOT_FINITE);
}

TEST(AttitudeTransformTest, ThrowsOnOutOfRangeValues)
{
    const beam::AttitudeConfig config;
    beam::RadarAttitude attitude;

    attitude = makeAttitude(0_deg, 0_deg, 0_deg);
    attitude.latitude = 91_deg;
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(attitude, config), beam::AttitudeErrorCode::OUT_OF_RANGE);

    attitude = makeAttitude(0_deg, 0_deg, 0_deg);
    attitude.longitude = -181_deg;
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(attitude, config), beam::AttitudeErrorCode::OUT_OF_RANGE);

    attitude = makeAttitude(0_deg, 0_deg, 0_deg);
    attitude.altitude = 101_km;
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(attitude, config), beam::AttitudeErrorCode::OUT_OF_RANGE);

    attitude = makeAttitude(181_deg, 0_deg, 0_deg);
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(attitude, config), beam::AttitudeErrorCode::OUT_OF_RANGE);

    attitude = makeAttitude(0_deg, 0_deg, 361_deg);
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(attitude, config), beam::AttitudeErrorCode::OUT_OF_RANGE);
}

TEST(AttitudeTransformTest, ValidatesAltitudeBoundariesWithDistanceLiterals)
{
    const beam::AttitudeConfig config;
    auto attitude = makeAttitude(0_deg, 0_deg, 0_deg);

    attitude.altitude = -0.5_km;
    EXPECT_NO_THROW(beam::AntEnuTransform(attitude, config));
    attitude.altitude = 100_km;
    EXPECT_NO_THROW(beam::AntEnuTransform(attitude, config));

    attitude.altitude = -0.501_km;
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(attitude, config), beam::AttitudeErrorCode::OUT_OF_RANGE);
    attitude.altitude = 100.001_km;
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(attitude, config), beam::AttitudeErrorCode::OUT_OF_RANGE);
}

TEST(AttitudeTransformTest, ThrowsOnGimbalLockPitch)
{
    const beam::AttitudeConfig config;

    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(makeAttitude(0_deg, 89_deg, 0_deg), config), beam::AttitudeErrorCode::GIMBAL_LOCK);
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(makeAttitude(0_deg, -95_deg, 0_deg), config), beam::AttitudeErrorCode::GIMBAL_LOCK);
    EXPECT_NO_THROW(beam::AntEnuTransform(makeAttitude(0_deg, 88.9_deg, 0_deg), config));
}

TEST(AttitudeTransformTest, AcceptsRotationMount)
{
    beam::AttitudeConfig config;
    config.mountAntToBodyRotation = math::SquareMatrix{{0, -1, 0}, {1, 0, 0}, {0, 0, 1}}; // z축 90도 회전

    EXPECT_NO_THROW(beam::AntEnuTransform(makeAttitude(0_deg, 0_deg, 0_deg), config));
}

TEST(AttitudeTransformTest, ThrowsOnInvalidMount)
{
    beam::AttitudeConfig config;
    const auto attitude = makeAttitude(0_deg, 0_deg, 0_deg);

    config.mountAntToBodyRotation = math::SquareMatrix{{2, 0, 0}, {0, 1, 0}, {0, 0, 1}}; // 스케일
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(attitude, config), beam::AttitudeErrorCode::INVALID_MOUNT);

    config.mountAntToBodyRotation = math::SquareMatrix{{1, 0, 0}, {0, 1, 0}, {0, 0, -1}}; // 반사 (det = -1)
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(attitude, config), beam::AttitudeErrorCode::INVALID_MOUNT);

    config.mountAntToBodyRotation = math::SquareMatrix{{1, 0, 0}, {0, 1, 0}, {0, 0, 0}}; // 특이 행렬
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(attitude, config), beam::AttitudeErrorCode::INVALID_MOUNT);

    config.mountAntToBodyRotation = math::SquareMatrix{{1, 0, 0}, {0, 1, 0}, {0, 0, std::numeric_limits<double>::infinity()}};
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(attitude, config), beam::AttitudeErrorCode::INVALID_MOUNT);

    config.mountAntToBodyRotation = math::SquareMatrix::identity(2); // 3x3 이 아님
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(attitude, config), beam::AttitudeErrorCode::INVALID_MOUNT);
}

TEST(AttitudeTransformTest, AcceptsRotationMountWithFloatingPointEntries)
{
    beam::AttitudeConfig config;
    const double c = std::cos(0.37);
    const double s = std::sin(0.37);
    config.mountAntToBodyRotation = math::SquareMatrix{{c, -s, 0}, {s, c, 0}, {0, 0, 1}};

    EXPECT_NO_THROW(beam::AntEnuTransform(makeAttitude(0_deg, 0_deg, 0_deg), config));
}

TEST(AttitudeTransformTest, MountOrthogonalityUsesConfiguredTolerance)
{
    beam::AttitudeConfig config;
    const auto attitude = makeAttitude(0_deg, 0_deg, 0_deg);

    config.mountAntToBodyRotation = math::SquareMatrix{{1.0 + 1e-7, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    EXPECT_NO_THROW(beam::AntEnuTransform(attitude, config));
    config.mountOrthogonalTolerance = 1e-8;
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(attitude, config), beam::AttitudeErrorCode::INVALID_MOUNT);

    config.mountAntToBodyRotation = math::SquareMatrix{{1, 1e-7, 0}, {0, 1, 0}, {0, 0, 1}};
    config.mountOrthogonalTolerance = 1e-7;
    EXPECT_NO_THROW(beam::AntEnuTransform(attitude, config));
    config.mountOrthogonalTolerance = 1e-8;
    EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(attitude, config), beam::AttitudeErrorCode::INVALID_MOUNT);
}

TEST(AttitudeTransformTest, RejectsNonFiniteMountEntriesAndOverflow)
{
    beam::AttitudeConfig config;
    const auto attitude = makeAttitude(0_deg, 0_deg, 0_deg);

    for (double value : {std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::max()})
    {
        config.mountAntToBodyRotation = math::SquareMatrix{{value, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(attitude, config), beam::AttitudeErrorCode::INVALID_MOUNT);
    }
}

TEST(AttitudeTransformTest, RejectsInvalidMountOrthogonalTolerance)
{
    beam::AttitudeConfig config;
    const auto attitude = makeAttitude(0_deg, 0_deg, 0_deg);

    config.mountOrthogonalTolerance = 0.0;
    EXPECT_NO_THROW(beam::AntEnuTransform(attitude, config));
    for (double tolerance : {-1.0, std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::infinity()})
    {
        config.mountOrthogonalTolerance = tolerance;
        EXPECT_ATTITUDE_ERROR(beam::AntEnuTransform(attitude, config), beam::AttitudeErrorCode::INVALID_MOUNT);
    }
}

TEST(AttitudeTransformTest, AttitudeErrorIsInvalidArgument)
{
    EXPECT_THROW(beam::AntEnuTransform(makeAttitude(0_deg, 95_deg, 0_deg), beam::AttitudeConfig{}), std::invalid_argument);
}

// ---------------------------------------------------------------- 각도 변환

TEST(AttitudeTransformTest, IdentityKeepsAngles)
{
    const beam::AntEnuTransform transform = makeValidTransform(0_deg, 0_deg, 0_deg);
    const auto [azimuth_enu, elevation_enu] = transform.antToEnu(-30_deg, 10_deg);
    EXPECT_NEAR(azimuth_enu.deg(), -30, 1e-9); // 음수 방위각이 0~360으로 바뀌지 않아야 함
    EXPECT_NEAR(elevation_enu.deg(), 10, 1e-9);
}

TEST(AttitudeTransformTest, YawRotatesAzimuthClockwise)
{
    const beam::AntEnuTransform transform = makeValidTransform(0_deg, 0_deg, 90_deg); // 정면이 동쪽
    const auto [azimuth_enu, elevation_enu] = transform.antToEnu(0_deg, 0_deg);
    EXPECT_NEAR(azimuth_enu.deg(), 90, 1e-9);
    EXPECT_NEAR(elevation_enu.deg(), 0, 1e-9);
}

TEST(AttitudeTransformTest, PitchRaisesBoresight)
{
    const beam::AntEnuTransform transform = makeValidTransform(0_deg, 10_deg, 0_deg);
    const auto [azimuth_enu, elevation_enu] = transform.antToEnu(0_deg, 0_deg);
    EXPECT_NEAR(azimuth_enu.deg(), 0, 1e-9);
    EXPECT_NEAR(elevation_enu.deg(), 10, 1e-9);
}

TEST(AttitudeTransformTest, AntToEnuAndBackRestoresAngles)
{
    const beam::AntEnuTransform transform = makeValidTransform(10_deg, 20_deg, 30_deg);

    for (math::Angle azimuth_ant : {-45.0_deg, -30.0_deg, 0.0_deg, 15.0_deg, 45.0_deg})
    {
        for (math::Angle elevation_ant : {0.0_deg, 10.0_deg, 41.2_deg})
        {
            const auto [azimuth_enu, elevation_enu] = transform.antToEnu(azimuth_ant, elevation_ant);
            const auto [azimuthRestored_ant, elevationRestored_ant] =
                transform.enuToAnt(azimuth_enu, elevation_enu);
            EXPECT_NEAR(azimuthRestored_ant.deg(), azimuth_ant.deg(), 1e-9);
            EXPECT_NEAR(elevationRestored_ant.deg(), elevation_ant.deg(), 1e-9);
        }
    }
}

TEST(AttitudeTransformTest, AzimuthIsWrappedToSignedRange)
{
    const beam::AntEnuTransform transform = makeValidTransform(0_deg, 0_deg, 0_deg);
    const auto [azimuthAbove180_enu, elevationAbove180_enu] = transform.antToEnu(181_deg, 0_deg);
    EXPECT_NEAR(azimuthAbove180_enu.deg(), -179, 1e-9);

    const auto [azimuthBelow180_enu, elevationBelow180_enu] = transform.antToEnu(-181_deg, 0_deg);
    EXPECT_NEAR(azimuthBelow180_enu.deg(), 179, 1e-9);

    const auto [azimuthAt180_enu, elevationAt180_enu] = transform.antToEnu(180_deg, 0_deg);
    EXPECT_NEAR(std::fabs(azimuthAt180_enu.deg()), 180, 1e-9); // 후방은 ±180 (0이 되면 안 됨)
}
