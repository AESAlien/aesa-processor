#include "attitude/ant_enu_transform.hpp"
#include "attitude/attitude_error.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <math/trigonometry.hpp>

using namespace math::literals;

namespace
{

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

    const auto direction_enu = transform.antToEnu(math::SphericalDirection(-30.0_deg, 10.0_deg));

    EXPECT_TRUE(direction_enu.azimuth().equals(-30_deg, 1e-9));
    EXPECT_TRUE(direction_enu.elevation().equals(10_deg, 1e-9));
}

TEST(AttitudeTransformTest, AcceptsAnglesCreatedFromRadians)
{
    const auto transform = makeValidTransform(0_deg, 0_deg, math::Angle::fromRadians(math::PI / 2.0));
    const auto direction_enu = transform.antToEnu(math::SphericalDirection(
        math::Angle::fromRadians(-math::PI / 6.0), math::Angle::fromRadians(math::PI / 18.0)));

    EXPECT_TRUE(direction_enu.azimuth().equals(60_deg, 1e-9));
    EXPECT_TRUE(direction_enu.elevation().equals(10_deg, 1e-9));
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
    const math::Angle angle = 0.37_rad;
    const double c = math::cos(angle);
    const double s = math::sin(angle);
    config.mountAntToBodyRotation = math::SquareMatrix{{c, -s, 0}, {s, c, 0}, {0, 0, 1}};

    EXPECT_NO_THROW(beam::AntEnuTransform(makeAttitude(0_deg, 0_deg, 0_deg), config));
}

TEST(AttitudeTransformTest, MountOrthogonalityUsesConfiguredTolerance)
{
    beam::AttitudeConfig config;
    const auto attitude = makeAttitude(0_deg, 0_deg, 0_deg);

    config.mountAntToBodyRotation = math::SquareMatrix{{1, 0.1, 0}, {0, 1, 0}, {0, 0, 1}};
    config.mountOrthogonalTolerance = 0.2;
    EXPECT_NO_THROW(beam::AntEnuTransform(attitude, config));
    config.mountOrthogonalTolerance = 0.05;
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
    const auto direction_enu = transform.antToEnu(math::SphericalDirection(-30_deg, 10_deg));
    EXPECT_TRUE(direction_enu.azimuth().equals(-30_deg, 1e-9)); // 음수 방위각이 0~360으로 바뀌지 않아야 함
    EXPECT_TRUE(direction_enu.elevation().equals(10_deg, 1e-9));
}

TEST(AttitudeTransformTest, YawRotatesAzimuthClockwise)
{
    const beam::AntEnuTransform transform = makeValidTransform(0_deg, 0_deg, 90_deg); // 정면이 동쪽
    const auto direction_enu = transform.antToEnu(math::SphericalDirection(0_deg, 0_deg));
    EXPECT_TRUE(direction_enu.azimuth().equals(90_deg, 1e-9));
    EXPECT_TRUE(direction_enu.elevation().equals(0_deg, 1e-9_deg));
}

TEST(AttitudeTransformTest, PitchRaisesBoresight)
{
    const beam::AntEnuTransform transform = makeValidTransform(0_deg, 10_deg, 0_deg);
    const auto direction_enu = transform.antToEnu(math::SphericalDirection(0_deg, 0_deg));
    EXPECT_TRUE(direction_enu.azimuth().equals(0_deg, 1e-9_deg));
    EXPECT_TRUE(direction_enu.elevation().equals(10_deg, 1e-9));
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
            EXPECT_TRUE(directionRestored_ant.azimuth().equals(azimuth_ant, 1e-9));
            EXPECT_TRUE(directionRestored_ant.elevation().equals(elevation_ant, 1e-9));
        }
    }
}

TEST(AttitudeTransformTest, AzimuthKeepsSignedRangeEndpoints)
{
    const beam::AntEnuTransform transform = makeValidTransform(0_deg, 0_deg, 0_deg);
    const auto directionAt180_enu = transform.antToEnu(math::SphericalDirection(180_deg, 0_deg));
    EXPECT_TRUE(math::abs(directionAt180_enu.azimuth()).equals(180_deg, 1e-9));
}
