#include <beam/domain/antenna_to_enu.hpp>
#include <beam/error/attitude_error.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{

constexpr double Tol = 1e-9;

beam::RadarAttitude MakeAttitude(double roll, double pitch, double yaw)
{
    beam::RadarAttitude a{};
    a.radarLat_deg = 37.0;
    a.radarLon_deg = 127.0;
    a.radarAlt_km = 0.1;
    a.roll_deg = roll;
    a.pitch_deg = pitch;
    a.yaw_deg = yaw;
    return a;
}

beam::AntennaToEnu MakeValid(double roll, double pitch, double yaw)
{
    return beam::AntennaToEnu(MakeAttitude(roll, pitch, yaw), beam::AttitudeConfig{});
}

} // namespace

// 실패 시 던지는 AttitudeError 의 code 를 확인하는 헬퍼
#define EXPECT_ATTITUDE_ERROR(expr, expectedCode)                                                  \
    do                                                                                             \
    {                                                                                              \
        try                                                                                        \
        {                                                                                          \
            (void)(expr);                                                                          \
            ADD_FAILURE() << "AttitudeError not thrown";                                           \
        }                                                                                          \
        catch (const beam::AttitudeError& e)                                                       \
        {                                                                                          \
            EXPECT_EQ(e.Code(), expectedCode);                                                     \
        }                                                                                          \
    } while (0)

// ---------------------------------------------------------------- makeAntennaToEnu

TEST(AttitudeTransformTest, ZeroAttitudeKeepsAngles)
{
    const beam::AntennaToEnu x = MakeValid(0, 0, 0);

    double az = 0.0;
    double el = 0.0;
    x.AntToEnuAngle(-30.0, 10.0, az, el);

    EXPECT_NEAR(az, -30.0, Tol);
    EXPECT_NEAR(el, 10.0, Tol);
}

TEST(AttitudeTransformTest, ThrowsOnNonFiniteValues)
{
    const beam::AttitudeConfig cfg;
    beam::RadarAttitude a = MakeAttitude(0, 0, 0);

    a.roll_deg = std::numeric_limits<double>::quiet_NaN();
    EXPECT_ATTITUDE_ERROR(beam::AntennaToEnu(a, cfg), beam::AttitudeErrorCode::NotFinite);

    a = MakeAttitude(0, 0, 0);
    a.radarAlt_km = std::numeric_limits<double>::infinity();
    EXPECT_ATTITUDE_ERROR(beam::AntennaToEnu(a, cfg), beam::AttitudeErrorCode::NotFinite);
}

TEST(AttitudeTransformTest, ThrowsOnOutOfRangeValues)
{
    const beam::AttitudeConfig cfg;
    beam::RadarAttitude a;

    a = MakeAttitude(0, 0, 0);
    a.radarLat_deg = 91;
    EXPECT_ATTITUDE_ERROR(beam::AntennaToEnu(a, cfg), beam::AttitudeErrorCode::OutOfRange);

    a = MakeAttitude(0, 0, 0);
    a.radarLon_deg = -181;
    EXPECT_ATTITUDE_ERROR(beam::AntennaToEnu(a, cfg), beam::AttitudeErrorCode::OutOfRange);

    a = MakeAttitude(0, 0, 0);
    a.radarAlt_km = 101;
    EXPECT_ATTITUDE_ERROR(beam::AntennaToEnu(a, cfg), beam::AttitudeErrorCode::OutOfRange);

    a = MakeAttitude(181, 0, 0);
    EXPECT_ATTITUDE_ERROR(beam::AntennaToEnu(a, cfg), beam::AttitudeErrorCode::OutOfRange);

    a = MakeAttitude(0, 0, 361);
    EXPECT_ATTITUDE_ERROR(beam::AntennaToEnu(a, cfg), beam::AttitudeErrorCode::OutOfRange);
}

TEST(AttitudeTransformTest, ThrowsOnGimbalLockPitch)
{
    const beam::AttitudeConfig cfg;

    EXPECT_ATTITUDE_ERROR(beam::AntennaToEnu(MakeAttitude(0, 89, 0), cfg),
                          beam::AttitudeErrorCode::GimbalLock);
    EXPECT_ATTITUDE_ERROR(beam::AntennaToEnu(MakeAttitude(0, -95, 0), cfg),
                          beam::AttitudeErrorCode::GimbalLock);
    EXPECT_NO_THROW(beam::AntennaToEnu(MakeAttitude(0, 88.9, 0), cfg));
}

TEST(AttitudeTransformTest, AcceptsRotationMount)
{
    beam::AttitudeConfig cfg;
    cfg.mountAntToBody = math::Matrix{{0, -1, 0}, {1, 0, 0}, {0, 0, 1}}; // z축 90도 회전

    EXPECT_NO_THROW(beam::AntennaToEnu(MakeAttitude(0, 0, 0), cfg));
}

TEST(AttitudeTransformTest, ThrowsOnInvalidMount)
{
    beam::AttitudeConfig cfg;
    const auto a = MakeAttitude(0, 0, 0);

    cfg.mountAntToBody = math::Matrix{{2, 0, 0}, {0, 1, 0}, {0, 0, 1}}; // 스케일
    EXPECT_ATTITUDE_ERROR(beam::AntennaToEnu(a, cfg), beam::AttitudeErrorCode::InvalidMount);

    cfg.mountAntToBody = math::Matrix{{1, 0, 0}, {0, 1, 0}, {0, 0, -1}}; // 반사 (det = -1)
    EXPECT_ATTITUDE_ERROR(beam::AntennaToEnu(a, cfg), beam::AttitudeErrorCode::InvalidMount);

    cfg.mountAntToBody = math::Matrix{{1, 0, 0}, {0, 1, 0}, {0, 0, 0}}; // 특이 행렬
    EXPECT_ATTITUDE_ERROR(beam::AntennaToEnu(a, cfg), beam::AttitudeErrorCode::InvalidMount);

    cfg.mountAntToBody =
        math::Matrix{{1, 0, 0}, {0, 1, 0}, {0, 0, std::numeric_limits<double>::infinity()}};
    EXPECT_ATTITUDE_ERROR(beam::AntennaToEnu(a, cfg), beam::AttitudeErrorCode::InvalidMount);

    cfg.mountAntToBody = math::Matrix::Identity(2); // 3x3 이 아님
    EXPECT_ATTITUDE_ERROR(beam::AntennaToEnu(a, cfg), beam::AttitudeErrorCode::InvalidMount);
}

TEST(AttitudeTransformTest, AttitudeErrorIsInvalidArgument)
{
    EXPECT_THROW(beam::AntennaToEnu(MakeAttitude(0, 95, 0), beam::AttitudeConfig{}),
                 std::invalid_argument);
}

// ---------------------------------------------------------------- 각도 변환

TEST(AttitudeTransformTest, IdentityKeepsAngles)
{
    const beam::AntennaToEnu x = MakeValid(0, 0, 0);
    double az = 0, el = 0;

    x.AntToEnuAngle(-30, 10, az, el);
    EXPECT_NEAR(az, -30, 1e-9); // 음수 방위각이 0~360으로 바뀌지 않아야 함
    EXPECT_NEAR(el, 10, 1e-9);
}

TEST(AttitudeTransformTest, YawRotatesAzimuthClockwise)
{
    const beam::AntennaToEnu x = MakeValid(0, 0, 90); // 정면이 동쪽
    double az = 0, el = 0;

    x.AntToEnuAngle(0, 0, az, el);
    EXPECT_NEAR(az, 90, 1e-9);
    EXPECT_NEAR(el, 0, 1e-9);
}

TEST(AttitudeTransformTest, PitchRaisesBoresight)
{
    const beam::AntennaToEnu x = MakeValid(0, 10, 0);
    double az = 0, el = 0;

    x.AntToEnuAngle(0, 0, az, el);
    EXPECT_NEAR(az, 0, 1e-9);
    EXPECT_NEAR(el, 10, 1e-9);
}

TEST(AttitudeTransformTest, AntToEnuAndBackRestoresAngles)
{
    const beam::AntennaToEnu x = MakeValid(10, 20, 30);

    for (double azIn : {-45.0, -30.0, 0.0, 15.0, 45.0})
    {
        for (double elIn : {0.0, 10.0, 41.2})
        {
            double azEnu = 0, elEnu = 0, azOut = 0, elOut = 0;
            x.AntToEnuAngle(azIn, elIn, azEnu, elEnu);
            x.EnuToAntAngle(azEnu, elEnu, azOut, elOut);
            EXPECT_NEAR(azOut, azIn, 1e-9);
            EXPECT_NEAR(elOut, elIn, 1e-9);
        }
    }
}

TEST(AttitudeTransformTest, AzimuthIsWrappedToSignedRange)
{
    const beam::AntennaToEnu x = MakeValid(0, 0, 0);
    double az = 0, el = 0;

    x.AntToEnuAngle(181, 0, az, el);
    EXPECT_NEAR(az, -179, 1e-9);

    x.AntToEnuAngle(-181, 0, az, el);
    EXPECT_NEAR(az, 179, 1e-9);

    x.AntToEnuAngle(180, 0, az, el);
    EXPECT_NEAR(std::fabs(az), 180, 1e-9); // 후방은 ±180 (0이 되면 안 됨)
}
