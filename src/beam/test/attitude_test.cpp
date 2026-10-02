#include <beam/domain/antenna_to_enu.hpp>
#include <beam/error/attitude_error.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{

constexpr double kTol = 1e-9;

beam::RadarAttitude makeAttitude(double roll, double pitch, double yaw)
{
    beam::RadarAttitude a{};
    a.radar_lat_deg = 37.0;
    a.radar_lon_deg = 127.0;
    a.radar_alt_km  = 0.1;
    a.roll_deg      = roll;
    a.pitch_deg     = pitch;
    a.yaw_deg       = yaw;
    return a;
}

beam::AntennaToEnu makeValid(double roll, double pitch, double yaw)
{
    return beam::AntennaToEnu(makeAttitude(roll, pitch, yaw), beam::AttitudeConfig{});
}

}   // namespace

// 실패 시 던지는 AttitudeError 의 code 를 확인하는 헬퍼
#define EXPECT_ATTITUDE_ERROR(expr, expected_code)                                   \
    do {                                                                             \
        try { (void)(expr); ADD_FAILURE() << "AttitudeError not thrown"; }           \
        catch(const beam::AttitudeError& e) { EXPECT_EQ(e.code(), expected_code); }  \
    } while(0)

// ---------------------------------------------------------------- makeAntennaToEnu

TEST(AttitudeTransformTest, ZeroAttitudeKeepsAngles)
{
    const beam::AntennaToEnu x = makeValid(0, 0, 0);

    double az = 0.0;
    double el = 0.0;
    x.antToEnuAngle(-30.0, 10.0, az, el);

    EXPECT_NEAR(az, -30.0, kTol);
    EXPECT_NEAR(el, 10.0, kTol);
}

TEST(AttitudeTransformTest, ThrowsOnNonFiniteValues)
{
    const beam::AttitudeConfig cfg;
    beam::RadarAttitude a = makeAttitude(0, 0, 0);

    a.roll_deg = std::numeric_limits<double>::quiet_NaN();
    EXPECT_ATTITUDE_ERROR(beam::AntennaToEnu(a, cfg), beam::AttitudeErrorCode::NotFinite);

    a = makeAttitude(0, 0, 0);
    a.radar_alt_km = std::numeric_limits<double>::infinity();
    EXPECT_ATTITUDE_ERROR(beam::AntennaToEnu(a, cfg), beam::AttitudeErrorCode::NotFinite);
}

TEST(AttitudeTransformTest, ThrowsOnOutOfRangeValues)
{
    const beam::AttitudeConfig cfg;
    beam::RadarAttitude a;

    a = makeAttitude(0, 0, 0); a.radar_lat_deg = 91;
    EXPECT_ATTITUDE_ERROR(beam::AntennaToEnu(a, cfg), beam::AttitudeErrorCode::OutOfRange);

    a = makeAttitude(0, 0, 0); a.radar_lon_deg = -181;
    EXPECT_ATTITUDE_ERROR(beam::AntennaToEnu(a, cfg), beam::AttitudeErrorCode::OutOfRange);

    a = makeAttitude(0, 0, 0); a.radar_alt_km = 101;
    EXPECT_ATTITUDE_ERROR(beam::AntennaToEnu(a, cfg), beam::AttitudeErrorCode::OutOfRange);

    a = makeAttitude(181, 0, 0);
    EXPECT_ATTITUDE_ERROR(beam::AntennaToEnu(a, cfg), beam::AttitudeErrorCode::OutOfRange);

    a = makeAttitude(0, 0, 361);
    EXPECT_ATTITUDE_ERROR(beam::AntennaToEnu(a, cfg), beam::AttitudeErrorCode::OutOfRange);
}

TEST(AttitudeTransformTest, ThrowsOnGimbalLockPitch)
{
    const beam::AttitudeConfig cfg;

    EXPECT_ATTITUDE_ERROR(beam::AntennaToEnu(makeAttitude(0, 89, 0), cfg),
                          beam::AttitudeErrorCode::GimbalLock);
    EXPECT_ATTITUDE_ERROR(beam::AntennaToEnu(makeAttitude(0, -95, 0), cfg),
                          beam::AttitudeErrorCode::GimbalLock);
    EXPECT_NO_THROW(beam::AntennaToEnu(makeAttitude(0, 88.9, 0), cfg));
}

TEST(AttitudeTransformTest, AcceptsRotationMount)
{
    beam::AttitudeConfig cfg;
    cfg.mount_ant_to_body = math::Matrix{{0,-1,0}, {1,0,0}, {0,0,1}};   // z축 90도 회전

    EXPECT_NO_THROW(beam::AntennaToEnu(makeAttitude(0, 0, 0), cfg));
}

TEST(AttitudeTransformTest, ThrowsOnInvalidMount)
{
    beam::AttitudeConfig cfg;
    const auto a = makeAttitude(0, 0, 0);

    cfg.mount_ant_to_body = math::Matrix{{2,0,0}, {0,1,0}, {0,0,1}};    // 스케일
    EXPECT_ATTITUDE_ERROR(beam::AntennaToEnu(a, cfg), beam::AttitudeErrorCode::InvalidMount);

    cfg.mount_ant_to_body = math::Matrix{{1,0,0}, {0,1,0}, {0,0,-1}};   // 반사 (det = -1)
    EXPECT_ATTITUDE_ERROR(beam::AntennaToEnu(a, cfg), beam::AttitudeErrorCode::InvalidMount);

    cfg.mount_ant_to_body = math::Matrix{{1,0,0}, {0,1,0}, {0,0,0}};    // 특이 행렬
    EXPECT_ATTITUDE_ERROR(beam::AntennaToEnu(a, cfg), beam::AttitudeErrorCode::InvalidMount);

    cfg.mount_ant_to_body = math::Matrix{{1,0,0}, {0,1,0}, {0,0,std::numeric_limits<double>::infinity()}};
    EXPECT_ATTITUDE_ERROR(beam::AntennaToEnu(a, cfg), beam::AttitudeErrorCode::InvalidMount);

    cfg.mount_ant_to_body = math::Matrix::Identity(2);                  // 3x3 이 아님
    EXPECT_ATTITUDE_ERROR(beam::AntennaToEnu(a, cfg), beam::AttitudeErrorCode::InvalidMount);
}

TEST(AttitudeTransformTest, AttitudeErrorIsInvalidArgument)
{
    EXPECT_THROW(beam::AntennaToEnu(makeAttitude(0, 95, 0), beam::AttitudeConfig{}),
                 std::invalid_argument);
}

// ---------------------------------------------------------------- 각도 변환

TEST(AttitudeTransformTest, IdentityKeepsAngles)
{
    const beam::AntennaToEnu x = makeValid(0, 0, 0);
    double az = 0, el = 0;

    x.antToEnuAngle(-30, 10, az, el);
    EXPECT_NEAR(az, -30, 1e-9);   // 음수 방위각이 0~360으로 바뀌지 않아야 함
    EXPECT_NEAR(el, 10, 1e-9);
}

TEST(AttitudeTransformTest, YawRotatesAzimuthClockwise)
{
    const beam::AntennaToEnu x = makeValid(0, 0, 90);   // 정면이 동쪽
    double az = 0, el = 0;

    x.antToEnuAngle(0, 0, az, el);
    EXPECT_NEAR(az, 90, 1e-9);
    EXPECT_NEAR(el, 0, 1e-9);
}

TEST(AttitudeTransformTest, PitchRaisesBoresight)
{
    const beam::AntennaToEnu x = makeValid(0, 10, 0);
    double az = 0, el = 0;

    x.antToEnuAngle(0, 0, az, el);
    EXPECT_NEAR(az, 0, 1e-9);
    EXPECT_NEAR(el, 10, 1e-9);
}

TEST(AttitudeTransformTest, AntToEnuAndBackRestoresAngles)
{
    const beam::AntennaToEnu x = makeValid(10, 20, 30);

    for(double az_in : {-45.0, -30.0, 0.0, 15.0, 45.0}) {
        for(double el_in : {0.0, 10.0, 41.2}) {
            double az_e = 0, el_e = 0, az_out = 0, el_out = 0;
            x.antToEnuAngle(az_in, el_in, az_e, el_e);
            x.enuToAntAngle(az_e, el_e, az_out, el_out);
            EXPECT_NEAR(az_out, az_in, 1e-9);
            EXPECT_NEAR(el_out, el_in, 1e-9);
        }
    }
}

TEST(AttitudeTransformTest, AzimuthIsWrappedToSignedRange)
{
    const beam::AntennaToEnu x = makeValid(0, 0, 0);
    double az = 0, el = 0;

    x.antToEnuAngle(181, 0, az, el);
    EXPECT_NEAR(az, -179, 1e-9);

    x.antToEnuAngle(-181, 0, az, el);
    EXPECT_NEAR(az, 179, 1e-9);

    x.antToEnuAngle(180, 0, az, el);
    EXPECT_NEAR(std::fabs(az), 180, 1e-9);   // 후방은 ±180 (0이 되면 안 됨)
}
