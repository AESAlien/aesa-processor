#include <beam/domain/antenna_to_enu.hpp>
#include <beam/error/attitude_error.hpp>

#include <math/matrix.hpp>
#include <math/angle.hpp>
#include <cmath>

namespace beam
{
namespace
{
math::Matrix bodyToEnu(double roll_deg, double pitch_deg, double yaw_deg)
{
    const double cr = std::cos(math::DegToRad(roll_deg));
    const double sr = std::sin(math::DegToRad(roll_deg));
    const double cp = std::cos(math::DegToRad(pitch_deg));
    const double sp = std::sin(math::DegToRad(pitch_deg));
    const double cy = std::cos(math::DegToRad(yaw_deg));
    const double sy = std::sin(math::DegToRad(yaw_deg));

    const math::Matrix Rz{ {cy, sy, 0}, {-sy, cy, 0}, {0, 0, 1} };
    const math::Matrix Rx{ {1, 0, 0}, {0, cp, -sp}, {0, sp, cp} };
    const math::Matrix Ry{ {cr, 0, sr}, {0, 1, 0}, {-sr, 0, cr} };

    return Rz * (Rx * Ry);
}

bool allFinite(const RadarAttitude& a)
{
    return std::isfinite(a.radar_lat_deg) && std::isfinite(a.radar_lon_deg)
        && std::isfinite(a.radar_alt_km)  && std::isfinite(a.roll_deg)
        && std::isfinite(a.pitch_deg)     && std::isfinite(a.yaw_deg);
}

bool isRotationMatrix(const math::Matrix& m, double tol)
{
    if(m.Rows() != 3 || m.Columns() != 3)
    {
        return false;
    }

    for(std::size_t i = 0; i < 3; ++i)
    {
        for(std::size_t j = 0; j < 3; ++j)
        {
            if(!std::isfinite(m(i, j)))
            {
                return false;
            }
        }
    }

    // R^T * R = I
    const math::Matrix rtr = m.Transpose() * m;
    for(std::size_t i = 0; i < 3; ++i)
    {
        for(std::size_t j = 0; j < 3; ++j)
        {
            if(std::fabs(rtr(i, j) - (i == j ? 1.0 : 0.0)) > tol)
            {
                return false;
            }
        }
    }

    const double det = m(0,0)*(m(1,1)*m(2,2) - m(1,2)*m(2,1))
                     - m(0,1)*(m(1,0)*m(2,2) - m(1,2)*m(2,0))
                     + m(0,2)*(m(1,0)*m(2,1) - m(1,1)*m(2,0));
    return std::fabs(det - 1.0) <= tol;
}

double clamp1(double x)
{
    return std::fmax(-1.0, std::fmin(1.0, x));
}

double wrap180(double d)
{
    d = std::fmod(d + 180.0, 360.0);
    if(d < 0.0)
    {
        d += 360.0;
    }
    return d - 180.0;
}

math::Matrix aedToVec(double az_deg, double el_deg)
{
    const double az = math::DegToRad(az_deg);
    const double el = math::DegToRad(el_deg);
    return math::Matrix{
        { std::cos(el) * std::sin(az) },
        { std::cos(el) * std::cos(az) },
        { std::sin(el) }
    };
}

void vecToAed(const math::Matrix& v, double& az_deg, double& el_deg)
{
    az_deg = wrap180(math::RadToDeg(std::atan2(v(0, 0), v(1, 0))));
    el_deg = math::RadToDeg(std::asin(clamp1(v(2, 0))));
}
}   // namespace

AntennaToEnu::AntennaToEnu(const RadarAttitude& a, const AttitudeConfig& cfg)
    : attitude_(a)
{
    if(!allFinite(a))
    {
        throw AttitudeError(AttitudeErrorCode::NotFinite, "RadarAttitude contains NaN or Inf");
    }

    if(a.radar_lat_deg < -90  || a.radar_lat_deg > 90  ||
       a.radar_lon_deg < -180 || a.radar_lon_deg > 180 ||
       a.radar_alt_km  < -0.5 || a.radar_alt_km  > 100 ||
       a.roll_deg      < -180 || a.roll_deg      > 180 ||
       a.yaw_deg       < -360 || a.yaw_deg       > 360)
    {
        throw AttitudeError(AttitudeErrorCode::OutOfRange, "RadarAttitude value is out of range");
    }

    if(std::fabs(a.pitch_deg) >= cfg.max_abs_pitch_deg)
    {
        throw AttitudeError(AttitudeErrorCode::GimbalLock, "pitch is too close to gimbal lock");
    }

    if(!isRotationMatrix(cfg.mount_ant_to_body, cfg.mount_ortho_tol))
    {
        throw AttitudeError(AttitudeErrorCode::InvalidMount, "mount_ant_to_body is not a rotation matrix");
    }

    const math::Matrix rotationMatrix = bodyToEnu(a.roll_deg, a.pitch_deg, a.yaw_deg)
                                      * cfg.mount_ant_to_body;
    rot_ant_to_enu_ = rotationMatrix;
}

void AntennaToEnu::antToEnuAngle(
    double az_ant_deg,
    double el_ant_deg,
    double& az_enu_deg,
    double& el_enu_deg) const
{
    const math::Matrix enu = rot_ant_to_enu_ * aedToVec(az_ant_deg, el_ant_deg);
    vecToAed(enu, az_enu_deg, el_enu_deg);
}

void AntennaToEnu::enuToAntAngle(
    double az_enu_deg,
    double el_enu_deg,
    double& az_ant_deg,
    double& el_ant_deg) const
{
    const math::Matrix antenna = rot_ant_to_enu_.Transpose() * aedToVec(az_enu_deg, el_enu_deg);
    vecToAed(antenna, az_ant_deg, el_ant_deg);
}

}   // namespace beam
