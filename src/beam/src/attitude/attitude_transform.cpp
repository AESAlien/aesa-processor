#include <beam/attitude_transform.hpp>
#include <math/matrix.hpp>
#include "rotation.hpp"
#include <cmath>

namespace beam
{
   
namespace
{
bool allFinite(const RadarAttitude& a)
{
    return std::isfinite(a.radar_lat_deg) && std::isfinite(a.radar_lon_deg)
        && std::isfinite(a.radar_alt_km)  && std::isfinite(a.roll_deg)
        && std::isfinite(a.pitch_deg)     && std::isfinite(a.yaw_deg);
}

math::Matrix toMatrix(const std::array<double,9>& v)
{
    return math::Matrix{
        { v[0], v[1], v[2] },
        { v[3], v[4], v[5] },
        { v[6], v[7], v[8] }
    };
}

std::array<double,9> toArray(const math::Matrix& m)
{
    return {
        m(0,0), m(0,1), m(0,2),
        m(1,0), m(1,1), m(1,2),
        m(2,0), m(2,1), m(2,2)
    };
}

bool isRotationMatrix(const std::array<double,9>& m, double tol)
{
    for(double v : m) { if(!std::isfinite(v)) { return false; } }

    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            // (R^T R)(i,j) = i열 · j열
            const double dot = m[0*3+i]*m[0*3+j] + m[1*3+i]*m[1*3+j] + m[2*3+i]*m[2*3+j];
            if(std::fabs(dot - (i == j ? 1.0 : 0.0)) > tol) { return false; }
        }
    }

    const double det = m[0]*(m[4]*m[8] - m[5]*m[7])
                     - m[1]*(m[3]*m[8] - m[5]*m[6])
                     + m[2]*(m[3]*m[7] - m[4]*m[6]);
    return std::fabs(det - 1.0) <= tol;
}
}   // namespace

AttitudeCheckResult makeAntennaToEnu(const RadarAttitude& a, 
    const AttitudeConfig& cfg, AntennaToEnu& out)
{
    if(!allFinite(a)) { out.valid = false; return AttitudeCheckResult::NotFinite; }

    if(a.radar_lat_deg < -90  || a.radar_lat_deg > 90  ||
       a.radar_lon_deg < -180 || a.radar_lon_deg > 180 ||
       a.radar_alt_km  < -0.5 || a.radar_alt_km  > 100 ||
       a.roll_deg      < -180 || a.roll_deg      > 180 ||
       a.yaw_deg       < -360 || a.yaw_deg       > 360)
       { out.valid = false; return AttitudeCheckResult::OutOfRange; }

    if(std::fabs(a.pitch_deg) >= cfg.max_abs_pitch_deg)
    { out.valid = false; return AttitudeCheckResult::GimbalLock; }

    if(!isRotationMatrix(cfg.mount_ant_to_body, cfg.mount_ortho_tol))
    { out.valid = false; return AttitudeCheckResult::InvalidMount; }

    const math::Matrix R_ant_to_enu =
        bodyToEnu(a.roll_deg, a.pitch_deg, a.yaw_deg) * toMatrix(cfg.mount_ant_to_body);

    out.att = a;
    out.rot_ant_to_enu = toArray(R_ant_to_enu);
    out.valid = true;
    return AttitudeCheckResult::Ok;
}

}   // namespace beam
