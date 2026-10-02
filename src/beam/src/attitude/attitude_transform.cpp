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

bool isRotationMatrix(const math::Matrix& m, double tol)
{
    if(m.Rows() != 3 || m.Columns() != 3) { return false; }

    for(std::size_t i = 0; i < 3; ++i) {
        for(std::size_t j = 0; j < 3; ++j) {
            if(!std::isfinite(m(i, j))) { return false; }
        }
    }

    // R^T * R = I
    const math::Matrix rtr = m.Transpose() * m;
    for(std::size_t i = 0; i < 3; ++i) {
        for(std::size_t j = 0; j < 3; ++j) {
            if(std::fabs(rtr(i, j) - (i == j ? 1.0 : 0.0)) > tol) { return false; }
        }
    }

    const double det = m(0,0)*(m(1,1)*m(2,2) - m(1,2)*m(2,1))
                     - m(0,1)*(m(1,0)*m(2,2) - m(1,2)*m(2,0))
                     + m(0,2)*(m(1,0)*m(2,1) - m(1,1)*m(2,0));
    return std::fabs(det - 1.0) <= tol;
}
}   // namespace

AntennaToEnu makeAntennaToEnu(const RadarAttitude& a, const AttitudeConfig& cfg)
{
    if(!allFinite(a))
    { throw AttitudeError(AttitudeErrorCode::NotFinite, "RadarAttitude contains NaN or Inf"); }

    if(a.radar_lat_deg < -90  || a.radar_lat_deg > 90  ||
       a.radar_lon_deg < -180 || a.radar_lon_deg > 180 ||
       a.radar_alt_km  < -0.5 || a.radar_alt_km  > 100 ||
       a.roll_deg      < -180 || a.roll_deg      > 180 ||
       a.yaw_deg       < -360 || a.yaw_deg       > 360)
    { throw AttitudeError(AttitudeErrorCode::OutOfRange, "RadarAttitude value is out of range"); }

    if(std::fabs(a.pitch_deg) >= cfg.max_abs_pitch_deg)
    { throw AttitudeError(AttitudeErrorCode::GimbalLock, "pitch is too close to gimbal lock"); }

    if(!isRotationMatrix(cfg.mount_ant_to_body, cfg.mount_ortho_tol))
    { throw AttitudeError(AttitudeErrorCode::InvalidMount, "mount_ant_to_body is not a rotation matrix"); }

    AntennaToEnu out;
    out.att = a;
    out.rot_ant_to_enu = bodyToEnu(a.roll_deg, a.pitch_deg, a.yaw_deg) * cfg.mount_ant_to_body;
    out.valid = true;
    return out;
}

}   // namespace beam
