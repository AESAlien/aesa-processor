#include <beam/attitude_transform.hpp>
#include <beam/transform.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

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
    beam::AntennaToEnu x;
    EXPECT_EQ(beam::makeAntennaToEnu(makeAttitude(roll, pitch, yaw), beam::AttitudeConfig{}, x),
              beam::AttitudeCheckResult::Ok);
    return x;
}

}   // namespace

// ---------------------------------------------------------------- makeAntennaToEnu

TEST(AttitudeTransformTest, ZeroAttitudeGivesIdentityMatrix)
{
    const beam::AntennaToEnu x = makeValid(0, 0, 0);

    ASSERT_TRUE(x.valid);
    const double identity[9] = {1,0,0, 0,1,0, 0,0,1};
    for(int i = 0; i < 9; ++i) { EXPECT_NEAR(x.rot_ant_to_enu[i], identity[i], kTol); }
}

TEST(AttitudeTransformTest, StoresInputAttitude)
{
    const beam::AntennaToEnu x = makeValid(10, 20, 30);

    EXPECT_DOUBLE_EQ(x.att.roll_deg, 10);
    EXPECT_DOUBLE_EQ(x.att.pitch_deg, 20);
    EXPECT_DOUBLE_EQ(x.att.yaw_deg, 30);
    EXPECT_DOUBLE_EQ(x.att.radar_lat_deg, 37.0);
}

TEST(AttitudeTransformTest, ResultIsProperRotationMatrix)
{
    const beam::AntennaToEnu x = makeValid(10, 20, 30);
    const auto& R = x.rot_ant_to_enu;

    // R^T * R = I
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            const double dot = R[0*3+i]*R[0*3+j] + R[1*3+i]*R[1*3+j] + R[2*3+i]*R[2*3+j];
            EXPECT_NEAR(dot, i == j ? 1.0 : 0.0, kTol);
        }
    }
    // det = +1
    const double det = R[0]*(R[4]*R[8] - R[5]*R[7])
                     - R[1]*(R[3]*R[8] - R[5]*R[6])
                     + R[2]*(R[3]*R[7] - R[4]*R[6]);
    EXPECT_NEAR(det, 1.0, kTol);
}

TEST(AttitudeTransformTest, RejectsNonFiniteValues)
{
    beam::AntennaToEnu x;
    beam::RadarAttitude a = makeAttitude(0, 0, 0);

    a.roll_deg = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(beam::makeAntennaToEnu(a, beam::AttitudeConfig{}, x), beam::AttitudeCheckResult::NotFinite);
    EXPECT_FALSE(x.valid);

    a = makeAttitude(0, 0, 0);
    a.radar_alt_km = std::numeric_limits<double>::infinity();
    EXPECT_EQ(beam::makeAntennaToEnu(a, beam::AttitudeConfig{}, x), beam::AttitudeCheckResult::NotFinite);
}

TEST(AttitudeTransformTest, RejectsOutOfRangeValues)
{
    beam::AntennaToEnu x;
    beam::RadarAttitude a;

    a = makeAttitude(0, 0, 0); a.radar_lat_deg = 91;
    EXPECT_EQ(beam::makeAntennaToEnu(a, beam::AttitudeConfig{}, x), beam::AttitudeCheckResult::OutOfRange);

    a = makeAttitude(0, 0, 0); a.radar_lon_deg = -181;
    EXPECT_EQ(beam::makeAntennaToEnu(a, beam::AttitudeConfig{}, x), beam::AttitudeCheckResult::OutOfRange);

    a = makeAttitude(0, 0, 0); a.radar_alt_km = 101;
    EXPECT_EQ(beam::makeAntennaToEnu(a, beam::AttitudeConfig{}, x), beam::AttitudeCheckResult::OutOfRange);

    a = makeAttitude(181, 0, 0);
    EXPECT_EQ(beam::makeAntennaToEnu(a, beam::AttitudeConfig{}, x), beam::AttitudeCheckResult::OutOfRange);

    a = makeAttitude(0, 0, 361);
    EXPECT_EQ(beam::makeAntennaToEnu(a, beam::AttitudeConfig{}, x), beam::AttitudeCheckResult::OutOfRange);
}

TEST(AttitudeTransformTest, RejectsGimbalLockPitch)
{
    beam::AntennaToEnu x;
    EXPECT_EQ(beam::makeAntennaToEnu(makeAttitude(0, 89, 0), beam::AttitudeConfig{}, x),
              beam::AttitudeCheckResult::GimbalLock);
    EXPECT_EQ(beam::makeAntennaToEnu(makeAttitude(0, -95, 0), beam::AttitudeConfig{}, x),
              beam::AttitudeCheckResult::GimbalLock);
    EXPECT_EQ(beam::makeAntennaToEnu(makeAttitude(0, 88.9, 0), beam::AttitudeConfig{}, x),
              beam::AttitudeCheckResult::Ok);
}

TEST(AttitudeTransformTest, AcceptsRotationMount)
{
    beam::AttitudeConfig cfg;
    cfg.mount_ant_to_body = {0,-1,0, 1,0,0, 0,0,1};   // z축 90도 회전

    beam::AntennaToEnu x;
    EXPECT_EQ(beam::makeAntennaToEnu(makeAttitude(0, 0, 0), cfg, x), beam::AttitudeCheckResult::Ok);
}

TEST(AttitudeTransformTest, RejectsInvalidMount)
{
    beam::AntennaToEnu x;
    beam::AttitudeConfig cfg;

    cfg.mount_ant_to_body = {2,0,0, 0,1,0, 0,0,1};    // 스케일
    EXPECT_EQ(beam::makeAntennaToEnu(makeAttitude(0, 0, 0), cfg, x), beam::AttitudeCheckResult::InvalidMount);

    cfg.mount_ant_to_body = {1,0,0, 0,1,0, 0,0,-1};   // 반사 (det = -1)
    EXPECT_EQ(beam::makeAntennaToEnu(makeAttitude(0, 0, 0), cfg, x), beam::AttitudeCheckResult::InvalidMount);

    cfg.mount_ant_to_body = {1,0,0, 0,1,0, 0,0,0};    // 특이 행렬
    EXPECT_EQ(beam::makeAntennaToEnu(makeAttitude(0, 0, 0), cfg, x), beam::AttitudeCheckResult::InvalidMount);

    cfg.mount_ant_to_body = {1,0,0, 0,1,0, 0,0,std::numeric_limits<double>::infinity()};
    EXPECT_EQ(beam::makeAntennaToEnu(makeAttitude(0, 0, 0), cfg, x), beam::AttitudeCheckResult::InvalidMount);
}

TEST(AttitudeTransformTest, FailureInvalidatesPreviousResult)
{
    beam::AntennaToEnu x = makeValid(0, 0, 0);
    ASSERT_TRUE(x.valid);

    EXPECT_NE(beam::makeAntennaToEnu(makeAttitude(0, 95, 0), beam::AttitudeConfig{}, x),
              beam::AttitudeCheckResult::Ok);
    EXPECT_FALSE(x.valid);
}

// ---------------------------------------------------------------- 각도 변환

TEST(AttitudeTransformTest, InvalidTransformReturnsFalse)
{
    const beam::AntennaToEnu x{};   // valid == false
    double az = 123.0, el = 456.0;

    EXPECT_FALSE(beam::antToEnuAngle(x, 0, 0, az, el));
    EXPECT_FALSE(beam::enuToAntAngle(x, 0, 0, az, el));
    EXPECT_DOUBLE_EQ(az, 123.0);   // 출력은 건드리지 않음
    EXPECT_DOUBLE_EQ(el, 456.0);
}

TEST(AttitudeTransformTest, IdentityKeepsAngles)
{
    const beam::AntennaToEnu x = makeValid(0, 0, 0);
    double az = 0, el = 0;

    ASSERT_TRUE(beam::antToEnuAngle(x, -30, 10, az, el));
    EXPECT_NEAR(az, -30, 1e-9);   // 음수 방위각이 0~360으로 바뀌지 않아야 함
    EXPECT_NEAR(el, 10, 1e-9);
}

TEST(AttitudeTransformTest, YawRotatesAzimuthClockwise)
{
    const beam::AntennaToEnu x = makeValid(0, 0, 90);   // 정면이 동쪽
    double az = 0, el = 0;

    ASSERT_TRUE(beam::antToEnuAngle(x, 0, 0, az, el));
    EXPECT_NEAR(az, 90, 1e-9);
    EXPECT_NEAR(el, 0, 1e-9);
}

TEST(AttitudeTransformTest, PitchRaisesBoresight)
{
    const beam::AntennaToEnu x = makeValid(0, 10, 0);
    double az = 0, el = 0;

    ASSERT_TRUE(beam::antToEnuAngle(x, 0, 0, az, el));
    EXPECT_NEAR(az, 0, 1e-9);
    EXPECT_NEAR(el, 10, 1e-9);
}

TEST(AttitudeTransformTest, AntToEnuAndBackRestoresAngles)
{
    const beam::AntennaToEnu x = makeValid(10, 20, 30);

    for(double az_in : {-45.0, -30.0, 0.0, 15.0, 45.0}) {
        for(double el_in : {0.0, 10.0, 41.2}) {
            double az_e = 0, el_e = 0, az_out = 0, el_out = 0;
            ASSERT_TRUE(beam::antToEnuAngle(x, az_in, el_in, az_e, el_e));
            ASSERT_TRUE(beam::enuToAntAngle(x, az_e, el_e, az_out, el_out));
            EXPECT_NEAR(az_out, az_in, 1e-9);
            EXPECT_NEAR(el_out, el_in, 1e-9);
        }
    }
}

TEST(AttitudeTransformTest, AzimuthIsWrappedToSignedRange)
{
    const beam::AntennaToEnu x = makeValid(0, 0, 0);
    double az = 0, el = 0;

    ASSERT_TRUE(beam::antToEnuAngle(x, 181, 0, az, el));
    EXPECT_NEAR(az, -179, 1e-9);

    ASSERT_TRUE(beam::antToEnuAngle(x, -181, 0, az, el));
    EXPECT_NEAR(az, 179, 1e-9);

    ASSERT_TRUE(beam::antToEnuAngle(x, 180, 0, az, el));
    EXPECT_NEAR(std::fabs(az), 180, 1e-9);   // 후방은 ±180 (0이 되면 안 됨)
}
