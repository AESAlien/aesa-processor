#include <attitude/updater.hpp>
#include <attitude/transform.hpp>
#include <math/angle.hpp>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <random>
#include <thread>
#include <vector>

using namespace attitude;

// ---------------------------------------------------------------------------
// 미니 테스트 프레임워크 (외부 라이브러리 없음)
// ---------------------------------------------------------------------------
namespace
{
struct TestCase { const char* name; void (*fn)(); };
std::vector<TestCase>& registry() { static std::vector<TestCase> r; return r; }
struct Registrar { Registrar(const char* n, void (*f)()) { registry().push_back({n, f}); } };
int g_cur_fail = 0;
}

#define TEST(name) \
    static void name(); \
    static Registrar reg_##name(#name, name); \
    static void name()

#define CHECK(cond) do { if (!(cond)) { \
    std::printf("    [FAIL] line %d: %s\n", __LINE__, #cond); ++g_cur_fail; } } while (0)

#define CHECK_NEAR(a, b, tol) do { \
    const double _a = (a), _b = (b); \
    if (!(std::fabs(_a - _b) <= (tol))) { \
        std::printf("    [FAIL] line %d: %s=%.9f  %s=%.9f (tol %g)\n", \
                    __LINE__, #a, _a, #b, _b, (double)(tol)); ++g_cur_fail; } } while (0)

// ---------------------------------------------------------------------------
// 헬퍼
// ---------------------------------------------------------------------------
namespace
{
constexpr std::int64_t kMs = 1'000'000LL;   // 1 ms in ns

// 방위각 차이를 [-180, 180] 로
double angDiff(double a, double b)
{
    double d = std::fmod(a - b, 360.0);
    if (d > 180.0)  d -= 360.0;
    if (d < -180.0) d += 360.0;
    return d;
}

struct Vec3 { double x, y, z; };

Vec3 toVec(double az_deg, double el_deg)
{
    const double az = math::DegToRad(az_deg);
    const double el = math::DegToRad(el_deg);
    return { std::cos(el) * std::sin(az), std::cos(el) * std::cos(az), std::sin(el) };
}

// 두 방향 사이의 각(deg). acos 는 각이 작을 때 정밀도가 나쁘므로 현(chord) 길이로 계산
double sepDeg(double az1, double el1, double az2, double el2)
{
    const Vec3 a = toVec(az1, el1), b = toVec(az2, el2);
    const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    const double chord = std::sqrt(dx * dx + dy * dy + dz * dz);
    return math::RadToDeg(2.0 * std::asin(std::fmin(1.0, chord / 2.0)));
}

AttitudeDto mkAtt(double roll, double pitch, double yaw)
{
    AttitudeDto a{};
    a.radar_lat_deg = 37.0; a.radar_lon_deg = 127.0; a.radar_alt_km = 0.05;
    a.roll_deg = roll; a.pitch_deg = pitch; a.yaw_deg = yaw;
    return a;
}

AttTransformDto makeXform(double roll, double pitch, double yaw, std::int64_t t = 1000)
{
    AttitudeStore s;
    s.update(mkAtt(roll, pitch, yaw), t);
    return s.snapshot(t);
}

// 회전행렬 R (행 우선) 이 정규직교 + det=+1 인지
bool isRotation(const std::array<double, 9>& R, double tol = 1e-12)
{
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            double s = 0;
            for (int k = 0; k < 3; ++k) s += R[i * 3 + k] * R[j * 3 + k];   // R * R^T
            if (std::fabs(s - (i == j ? 1.0 : 0.0)) > tol) return false;
        }
    const double det = R[0] * (R[4] * R[8] - R[5] * R[7])
                     - R[1] * (R[3] * R[8] - R[5] * R[6])
                     + R[2] * (R[3] * R[7] - R[4] * R[6]);
    return std::fabs(det - 1.0) <= tol;
}
}   // namespace

// ===========================================================================
// A. 알려진 값 (손으로 계산 가능한 케이스)
// ===========================================================================
TEST(identity_attitude_passes_angles_through)
{
    const auto x = makeXform(0, 0, 0);
    for (double az : {0.0, 45.0, 90.0, 180.0, 270.0, 359.0})
        for (double el : {-60.0, 0.0, 30.0, 85.0}) {
            double a, e;
            CHECK(antToEnuAngle(x, az, el, a, e));
            CHECK_NEAR(angDiff(a, az), 0.0, 1e-9);
            CHECK_NEAR(e, el, 1e-9);
        }
}

TEST(yaw_cardinal_directions)
{
    // yaw = 기수 방위. 안테나 정면(az 0)은 ENU 방위 = yaw 가 되어야 함
    for (double yaw : {0.0, 90.0, 180.0, 270.0, -90.0, 360.0, 45.0}) {
        const auto x = makeXform(0, 0, yaw);
        double a, e;
        antToEnuAngle(x, 0.0, 0.0, a, e);
        CHECK_NEAR(angDiff(a, yaw), 0.0, 1e-9);
        CHECK_NEAR(e, 0.0, 1e-9);
    }
}

TEST(pitch_raises_boresight_elevation)
{
    for (double p : {-60.0, -10.0, 10.0, 45.0, 80.0}) {
        const auto x = makeXform(0, p, 0);
        double a, e;
        antToEnuAngle(x, 0.0, 0.0, a, e);
        CHECK_NEAR(e, p, 1e-9);
        CHECK_NEAR(angDiff(a, 0.0), 0.0, 1e-9);
    }
}

TEST(roll_right_wing_down)
{
    // roll=90: 안테나 우측(az 90, el 0) 이 아래(-90)를 향함
    const auto x = makeXform(90, 0, 0);
    double a, e;
    antToEnuAngle(x, 90.0, 0.0, a, e);
    CHECK_NEAR(e, -90.0, 1e-9);
    // roll=-90: 우측이 위로
    const auto y = makeXform(-90, 0, 0);
    antToEnuAngle(y, 90.0, 0.0, a, e);
    CHECK_NEAR(e, 90.0, 1e-9);
}

TEST(yaw_and_pitch_combined)
{
    const auto x = makeXform(0, 30, 90);   // 동쪽을 보며 기수 30도 상승
    double a, e;
    antToEnuAngle(x, 0.0, 0.0, a, e);
    CHECK_NEAR(angDiff(a, 90.0), 0.0, 1e-9);
    CHECK_NEAR(e, 30.0, 1e-9);
}

TEST(rotation_order_roll_then_pitch_then_yaw)
{
    // roll=90, yaw=90: 안테나 상방(el 90)은 roll 로 우측(body +x) 이 되고,
    // yaw 90 에 의해 우측은 남쪽(az 180) 을 향함
    const auto x = makeXform(90, 0, 90);
    double a, e;
    antToEnuAngle(x, 0.0, 90.0, a, e);
    CHECK_NEAR(angDiff(a, 180.0), 0.0, 1e-9);
    CHECK_NEAR(e, 0.0, 1e-9);
}

TEST(zenith_and_nadir_stay_finite)
{
    std::mt19937 rng(1);
    std::uniform_real_distribution<double> u(-80, 80), y(0, 360);
    for (int i = 0; i < 500; ++i) {
        const auto x = makeXform(u(rng), u(rng), y(rng));
        for (double el : {90.0, -90.0}) {
            double a, e;
            CHECK(antToEnuAngle(x, 123.0, el, a, e));
            CHECK(std::isfinite(a) && std::isfinite(e));
            CHECK(e >= -90.0 && e <= 90.0);
        }
    }
}

// ===========================================================================
// B. 수학적 성질 (랜덤, seed 고정)
// ===========================================================================
TEST(rotation_matrix_is_orthonormal_and_proper)
{
    std::mt19937 rng(2);
    std::uniform_real_distribution<double> r(-180, 180), p(-88, 88), y(-360, 360);
    for (int i = 0; i < 2000; ++i) {
        const auto x = makeXform(r(rng), p(rng), y(rng));
        CHECK(x.valid);
        CHECK(isRotation(x.rot_ant_to_enu));
    }
}

TEST(roundtrip_ant_enu_ant)
{
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> r(-180, 180), p(-88, 88), y(0, 360),
                                           az(-720, 720), el(-89.9, 89.9);
    for (int i = 0; i < 5000; ++i) {
        const auto x = makeXform(r(rng), p(rng), y(rng));
        const double a0 = az(rng), e0 = el(rng);
        double ea, ee, ba, be;
        CHECK(antToEnuAngle(x, a0, e0, ea, ee));
        CHECK(enuToAntAngle(x, ea, ee, ba, be));
        CHECK_NEAR(sepDeg(ba, be, a0, e0), 0.0, 1e-6);   // 극점 근처 az 불안정 -> 방향 차이로 비교
    }
}

TEST(roundtrip_enu_ant_enu)
{
    std::mt19937 rng(4);
    std::uniform_real_distribution<double> r(-180, 180), p(-88, 88), y(0, 360),
                                           az(0, 360), el(-89.9, 89.9);
    for (int i = 0; i < 5000; ++i) {
        const auto x = makeXform(r(rng), p(rng), y(rng));
        const double a0 = az(rng), e0 = el(rng);
        double aa, ae, ba, be;
        enuToAntAngle(x, a0, e0, aa, ae);
        antToEnuAngle(x, aa, ae, ba, be);
        CHECK_NEAR(sepDeg(ba, be, a0, e0), 0.0, 1e-6);
    }
}

TEST(rotation_preserves_angle_between_directions)
{
    std::mt19937 rng(5);
    std::uniform_real_distribution<double> r(-180, 180), p(-88, 88), y(0, 360),
                                           az(0, 360), el(-89, 89);
    for (int i = 0; i < 3000; ++i) {
        const auto x = makeXform(r(rng), p(rng), y(rng));
        const double a1 = az(rng), e1 = el(rng), a2 = az(rng), e2 = el(rng);
        double A1, E1, A2, E2;
        antToEnuAngle(x, a1, e1, A1, E1);
        antToEnuAngle(x, a2, e2, A2, E2);
        CHECK_NEAR(sepDeg(a1, e1, a2, e2), sepDeg(A1, E1, A2, E2), 1e-6);
    }
}

TEST(output_ranges)
{
    std::mt19937 rng(6);
    std::uniform_real_distribution<double> r(-180, 180), p(-88, 88), y(0, 360),
                                           az(-1000, 1000), el(-90, 90);
    for (int i = 0; i < 3000; ++i) {
        const auto x = makeXform(r(rng), p(rng), y(rng));
        double a, e;
        antToEnuAngle(x, az(rng), el(rng), a, e);
        CHECK(a >= 0.0 && a < 360.0);
        CHECK(e >= -90.0 && e <= 90.0);
        enuToAntAngle(x, az(rng), el(rng), a, e);
        CHECK(a >= 0.0 && a < 360.0);
        CHECK(e >= -90.0 && e <= 90.0);
    }
}

TEST(az_input_wraps_by_360)
{
    const auto x = makeXform(10, 5, 40);
    double a1, e1, a2, e2, a3, e3;
    antToEnuAngle(x, 30.0, 10.0, a1, e1);
    antToEnuAngle(x, 30.0 + 360.0, 10.0, a2, e2);
    antToEnuAngle(x, 30.0 - 720.0, 10.0, a3, e3);
    CHECK_NEAR(angDiff(a1, a2), 0.0, 1e-9);
    CHECK_NEAR(angDiff(a1, a3), 0.0, 1e-9);
    CHECK_NEAR(e1, e2, 1e-9);
    CHECK_NEAR(e1, e3, 1e-9);
}

// ===========================================================================
// C. 운용 시나리오
// ===========================================================================
TEST(scenario_platform_turning_fixed_enu_target)
{
    // 수평 자세에서 플랫폼이 한 바퀴 돌 때, ENU 방위 45 의 표적은
    // 안테나 방위 = 45 - yaw 로 보여야 함
    for (int yaw = 0; yaw < 360; ++yaw) {
        const auto x = makeXform(0, 0, yaw);
        double a, e;
        CHECK(enuToAntAngle(x, 45.0, 0.0, a, e));
        CHECK_NEAR(angDiff(a, 45.0 - yaw), 0.0, 1e-9);
        CHECK_NEAR(e, 0.0, 1e-9);
    }
}

TEST(scenario_ship_rolling_pitching_beam_stays_on_target)
{
    // 선박 롤/피치 흔들림 중에도 ENU 표적 -> 빔 각도 -> ENU 복원이 표적과 일치
    const double tgt_az = 200.0, tgt_el = 15.0;
    for (int t = 0; t < 360; t += 3) {
        const double roll  = 20.0 * std::sin(math::DegToRad(t));
        const double pitch = 10.0 * std::sin(math::DegToRad(2 * t));
        const auto x = makeXform(roll, pitch, 80.0);
        double ba, be, ra, re;
        enuToAntAngle(x, tgt_az, tgt_el, ba, be);
        antToEnuAngle(x, ba, be, ra, re);
        CHECK_NEAR(angDiff(ra, tgt_az), 0.0, 1e-8);
        CHECK_NEAR(re, tgt_el, 1e-8);
    }
}

TEST(scenario_mount_offset_applied)
{
    // 안테나 정면이 플랫폼 우측(+x)을 향하도록 장착 (안테나 az 0 -> body az 90)
    AttitudeConfig cfg;
    cfg.mount_ant_to_body = { 0, 1, 0,
                             -1, 0, 0,
                              0, 0, 1 };
    AttitudeStore s(cfg);
    s.update(mkAtt(0, 0, 0), 1000);
    const auto x = s.snapshot(1000);
    CHECK(isRotation(x.rot_ant_to_enu));
    double a, e;
    antToEnuAngle(x, 0.0, 0.0, a, e);
    CHECK_NEAR(angDiff(a, 90.0), 0.0, 1e-9);

    // yaw 90 이면 안테나는 남쪽(180)을 봄
    s.update(mkAtt(0, 0, 90), 2000);
    antToEnuAngle(s.snapshot(2000), 0.0, 0.0, a, e);
    CHECK_NEAR(angDiff(a, 180.0), 0.0, 1e-9);
}

// ===========================================================================
// D. AttitudeStore: 입력 검증
// ===========================================================================
TEST(never_updated_store_is_invalid)
{
    AttitudeStore s;
    const auto x = s.snapshot(0);
    CHECK(!x.valid);
    CHECK(x.update_seq == 0);
    CHECK(s.rejectCount() == 0);
}

TEST(invalid_transform_refuses_conversion_and_keeps_outputs)
{
    const AttTransformDto invalid{};   // valid == false
    double a = 111.0, e = 222.0;
    CHECK(!antToEnuAngle(invalid, 10, 10, a, e));
    CHECK(a == 111.0 && e == 222.0);
    CHECK(!enuToAntAngle(invalid, 10, 10, a, e));
    CHECK(a == 111.0 && e == 222.0);
}

TEST(reject_nan_and_inf_in_every_field)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (int field = 0; field < 6; ++field)
        for (double bad : {nan, inf, -inf}) {
            AttitudeStore s;
            s.update(mkAtt(1, 2, 3), 100);
            AttitudeDto a = mkAtt(1, 2, 3);
            double* p[6] = { &a.radar_lat_deg, &a.radar_lon_deg, &a.radar_alt_km,
                             &a.roll_deg, &a.pitch_deg, &a.yaw_deg };
            *p[field] = bad;
            CHECK(s.update(a, 200) == AttUpdateResult::NotFinite);
            CHECK(s.rejectCount() == 1);
            const auto x = s.snapshot(100);
            CHECK(x.valid && x.update_seq == 1 && x.att.yaw_deg == 3.0);   // 이전 값 유지
        }
}

TEST(reject_out_of_range_values)
{
    struct Case { const char* name; AttitudeDto a; };
    std::vector<Case> cases;
    { auto a = mkAtt(0, 0, 0); a.radar_lat_deg =  90.001; cases.push_back({"lat+", a}); }
    { auto a = mkAtt(0, 0, 0); a.radar_lat_deg = -90.001; cases.push_back({"lat-", a}); }
    { auto a = mkAtt(0, 0, 0); a.radar_lon_deg =  180.001; cases.push_back({"lon+", a}); }
    { auto a = mkAtt(0, 0, 0); a.radar_lon_deg = -180.001; cases.push_back({"lon-", a}); }
    { auto a = mkAtt(0, 0, 0); a.radar_alt_km  =  100.001; cases.push_back({"alt+", a}); }
    { auto a = mkAtt(0, 0, 0); a.radar_alt_km  = -0.501;   cases.push_back({"alt-", a}); }
    { auto a = mkAtt(180.001, 0, 0);  cases.push_back({"roll+", a}); }
    { auto a = mkAtt(-180.001, 0, 0); cases.push_back({"roll-", a}); }
    { auto a = mkAtt(0, 0, 360.001);  cases.push_back({"yaw+", a}); }
    { auto a = mkAtt(0, 0, -360.001); cases.push_back({"yaw-", a}); }
    for (const auto& c : cases) {
        AttitudeStore s;
        const auto r = s.update(c.a, 100);
        if (r != AttUpdateResult::OutOfRange) std::printf("    case %s\n", c.name);
        CHECK(r == AttUpdateResult::OutOfRange);
        CHECK(!s.snapshot(100).valid);
        CHECK(s.rejectCount() == 1);
    }
}

TEST(accept_boundary_values)
{
    struct Case { const char* name; AttitudeDto a; };
    std::vector<Case> cases;
    { auto a = mkAtt(0, 0, 0); a.radar_lat_deg =  90;  cases.push_back({"lat=90", a}); }
    { auto a = mkAtt(0, 0, 0); a.radar_lat_deg = -90;  cases.push_back({"lat=-90", a}); }
    { auto a = mkAtt(0, 0, 0); a.radar_lon_deg =  180; cases.push_back({"lon=180", a}); }
    { auto a = mkAtt(0, 0, 0); a.radar_lon_deg = -180; cases.push_back({"lon=-180", a}); }
    { auto a = mkAtt(0, 0, 0); a.radar_alt_km  =  100; cases.push_back({"alt=100", a}); }
    { auto a = mkAtt(0, 0, 0); a.radar_alt_km  = -0.5; cases.push_back({"alt=-0.5", a}); }
    { auto a = mkAtt( 180, 0, 0); cases.push_back({"roll=180", a}); }
    { auto a = mkAtt(-180, 0, 0); cases.push_back({"roll=-180", a}); }
    { auto a = mkAtt(0, 0,  360); cases.push_back({"yaw=360", a}); }
    { auto a = mkAtt(0, 0, -360); cases.push_back({"yaw=-360", a}); }
    { auto a = mkAtt(0,  88.99, 0); cases.push_back({"pitch=88.99", a}); }
    { auto a = mkAtt(0, -88.99, 0); cases.push_back({"pitch=-88.99", a}); }
    for (const auto& c : cases) {
        AttitudeStore s;
        const auto r = s.update(c.a, 100);
        if (r != AttUpdateResult::Ok) std::printf("    case %s\n", c.name);
        CHECK(r == AttUpdateResult::Ok);
        CHECK(s.snapshot(100).valid);
        CHECK(s.rejectCount() == 0);
    }
}

TEST(gimbal_lock_limit_is_configurable)
{
    { AttitudeStore s;   // 기본 89도
      CHECK(s.update(mkAtt(0,  89.0, 0), 1) == AttUpdateResult::GimbalLock);
      CHECK(s.update(mkAtt(0, -89.0, 0), 1) == AttUpdateResult::GimbalLock);
      CHECK(s.update(mkAtt(0,  90.0, 0), 1) == AttUpdateResult::GimbalLock);
      CHECK(s.update(mkAtt(0,  88.9, 0), 1) == AttUpdateResult::Ok); }
    { AttitudeConfig c; c.max_abs_pitch_deg = 30.0;
      AttitudeStore s(c);
      CHECK(s.update(mkAtt(0, 29.9, 0), 1) == AttUpdateResult::Ok);
      CHECK(s.update(mkAtt(0, 30.0, 0), 2) == AttUpdateResult::GimbalLock); }
}

// ===========================================================================
// E. AttitudeStore: update_seq, 시각, stale
// ===========================================================================
TEST(update_seq_increments_only_on_success)
{
    AttitudeStore s;
    CHECK(s.snapshot(0).update_seq == 0);
    s.update(mkAtt(0, 0, 1), 10);
    CHECK(s.snapshot(10).update_seq == 1);
    s.update(mkAtt(0, 0, 2), 20);
    CHECK(s.snapshot(20).update_seq == 2);
    s.update(mkAtt(0, 95, 3), 30);                     // 거부
    CHECK(s.snapshot(20).update_seq == 2);
    s.update(mkAtt(0, 0, 4), 40);
    CHECK(s.snapshot(40).update_seq == 3);
    CHECK(s.snapshot(40).att.yaw_deg == 4.0);
    CHECK(s.rejectCount() == 1);
}

TEST(snapshot_stores_rx_time_and_att)
{
    AttitudeStore s;
    const AttitudeDto in = mkAtt(5, -6, 7);
    s.update(in, 123456789LL);
    const auto x = s.snapshot(123456789LL);
    CHECK(x.rx_time_ns == 123456789LL);
    CHECK(x.att.roll_deg == 5.0 && x.att.pitch_deg == -6.0 && x.att.yaw_deg == 7.0);
    CHECK(x.att.radar_lat_deg == in.radar_lat_deg);
    CHECK(x.att.radar_lon_deg == in.radar_lon_deg);
    CHECK(x.att.radar_alt_km  == in.radar_alt_km);
}

TEST(stale_boundary_default_200ms)
{
    AttitudeStore s;
    s.update(mkAtt(0, 0, 0), 1000 * kMs);
    CHECK( s.snapshot(1000 * kMs).valid);          // 0 ms
    CHECK( s.snapshot(1199 * kMs).valid);          // 199 ms
    CHECK( s.snapshot(1200 * kMs).valid);          // 정확히 200 ms: 유효(초과해야 stale)
    CHECK(!s.snapshot(1200 * kMs + 1).valid);      // 200 ms + 1 ns
    CHECK(!s.snapshot(5000 * kMs).valid);
}

TEST(stale_is_temporary_new_update_recovers)
{
    AttitudeStore s;
    s.update(mkAtt(0, 0, 10), 0);
    CHECK(!s.snapshot(500 * kMs).valid);
    s.update(mkAtt(0, 0, 20), 500 * kMs);
    const auto x = s.snapshot(510 * kMs);
    CHECK(x.valid);
    CHECK(x.att.yaw_deg == 20.0);
}

TEST(stale_uses_custom_max_age)
{
    AttitudeConfig c; c.max_age_ms = 50;
    AttitudeStore s(c);
    s.update(mkAtt(0, 0, 0), 0);
    CHECK( s.snapshot(50 * kMs).valid);
    CHECK(!s.snapshot(51 * kMs).valid);
}

TEST(stale_does_not_modify_stored_data_only_flag)
{
    AttitudeStore s;
    s.update(mkAtt(1, 2, 3), 0);
    const auto stale = s.snapshot(10'000 * kMs);
    CHECK(!stale.valid);
    CHECK(stale.att.yaw_deg == 3.0 && stale.update_seq == 1);
    double a = 5, e = 5;
    CHECK(!antToEnuAngle(stale, 0, 0, a, e));      // stale 이면 변환 거부
}

TEST(clock_skew_now_before_rx_is_treated_as_fresh)
{
    // now < rx_time (시계 기준 불일치) 이면 age 가 음수 -> 현재 구현은 유효 처리.
    // 이 동작이 의도가 아니라면 이 테스트를 뒤집고 구현을 바꿀 것.
    AttitudeStore s;
    s.update(mkAtt(0, 0, 0), 1000 * kMs);
    CHECK(s.snapshot(0).valid);
}

// ===========================================================================
// F. pollOnce + Mock Port
// ===========================================================================
namespace
{
class MockPort : public AttitudePort
{
public:
    AttQueStatus status = AttQueStatus::Success;
    AttitudeDto  dto{};
    std::int64_t rx = 0;
    int          calls = 0;
    std::chrono::milliseconds last_timeout{0};

    AttQueStatus read(AttitudeDto& out, std::int64_t& out_rx,
                      std::chrono::milliseconds timeout) override
    {
        ++calls;
        last_timeout = timeout;
        if (status == AttQueStatus::Success) { out = dto; out_rx = rx; }
        return status;
    }
};
}

TEST(pollOnce_success_updates_store)
{
    MockPort port; port.dto = mkAtt(1, 2, 3); port.rx = 777;
    AttitudeStore s;
    AttUpdateResult r = AttUpdateResult::NotFinite;
    CHECK(pollOnce(port, s, std::chrono::milliseconds(10), &r) == AttQueStatus::Success);
    CHECK(r == AttUpdateResult::Ok);
    CHECK(port.calls == 1);
    CHECK(port.last_timeout == std::chrono::milliseconds(10));
    const auto x = s.snapshot(777);
    CHECK(x.valid && x.rx_time_ns == 777 && x.att.yaw_deg == 3.0);
}

TEST(pollOnce_timeout_and_closed_do_not_touch_store)
{
    AttitudeStore s;
    s.update(mkAtt(0, 0, 9), 100);
    MockPort port;
    port.dto = mkAtt(0, 0, 55);

    port.status = AttQueStatus::Timeout;
    CHECK(pollOnce(port, s, std::chrono::milliseconds(1)) == AttQueStatus::Timeout);
    port.status = AttQueStatus::Closed;
    CHECK(pollOnce(port, s, std::chrono::milliseconds(1)) == AttQueStatus::Closed);

    const auto x = s.snapshot(100);
    CHECK(x.att.yaw_deg == 9.0 && x.update_seq == 1);
}

TEST(pollOnce_reports_rejection_and_null_out_res_is_ok)
{
    MockPort port; port.dto = mkAtt(0, 95, 0); port.rx = 5;   // 짐벌락
    AttitudeStore s;
    AttUpdateResult r = AttUpdateResult::Ok;
    CHECK(pollOnce(port, s, std::chrono::milliseconds(1), &r) == AttQueStatus::Success);
    CHECK(r == AttUpdateResult::GimbalLock);
    CHECK(s.rejectCount() == 1);
    CHECK(pollOnce(port, s, std::chrono::milliseconds(1), nullptr) == AttQueStatus::Success);
    CHECK(s.rejectCount() == 2);
}

// ===========================================================================
// G. 동시성: 쓰기 스레드 1 + 읽기 스레드 3 에서 찢어진 읽기가 없는지
//    (ThreadSanitizer 로 돌리면 더 좋음: -fsanitize=thread)
// ===========================================================================
TEST(concurrent_update_and_snapshot_is_consistent)
{
    AttitudeStore s;
    std::atomic<bool> stop{false};
    std::atomic<int>  torn{0};
    std::atomic<long> reads{0};

    // roll = pitch = yaw = k 로 쓰면, 정면 방향의 ENU 결과는 항상 (az=k, el=k)
    // -> att 와 회전행렬이 서로 다른 갱신에서 섞이면 감지됨
    auto reader = [&] {
        std::uint32_t last_seq = 0;
        while (!stop.load()) {
            const auto x = s.snapshot(nowNs());
            if (!x.valid) continue;
            ++reads;
            const double k = x.att.yaw_deg;
            if (x.att.roll_deg != k || x.att.pitch_deg != k) ++torn;
            double a, e;
            antToEnuAngle(x, 0.0, 0.0, a, e);
            if (std::fabs(angDiff(a, k)) > 1e-6 || std::fabs(e - k) > 1e-6) ++torn;
            if (x.update_seq < last_seq) ++torn;       // seq 는 단조 증가
            last_seq = x.update_seq;
        }
    };

    std::thread r1(reader), r2(reader), r3(reader);
    for (int i = 0; i < 20000; ++i) {
        const double k = static_cast<double>(i % 80);
        s.update(mkAtt(k, k, k), nowNs());
    }
    stop = true;
    r1.join(); r2.join(); r3.join();

    CHECK(torn.load() == 0);
    CHECK(reads.load() > 0);
    CHECK(s.snapshot(nowNs()).update_seq == 20000u);
}

// ===========================================================================
int main()
{
    int failed_tests = 0;
    for (const auto& t : registry()) {
        g_cur_fail = 0;
        t.fn();
        if (g_cur_fail == 0) {
            std::printf("[ PASS ] %s\n", t.name);
        } else {
            std::printf("[ FAIL ] %s (%d checks)\n", t.name, g_cur_fail);
            ++failed_tests;
        }
    }
    std::printf("\n%zu tests, %d failed\n", registry().size(), failed_tests);
    return failed_tests ? 1 : 0;
}