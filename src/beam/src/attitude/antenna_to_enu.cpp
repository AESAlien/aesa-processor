#include <beam/domain/antenna_to_enu.hpp>
#include <beam/error/attitude_error.hpp>

#include <cmath>
#include <math/angle.hpp>
#include <math/matrix.hpp>

namespace beam
{
namespace
{
math::Matrix BodyToEnu(double roll_deg, double pitch_deg, double yaw_deg)
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

    return rz * (rx * ry);
}

bool AllFinite(const RadarAttitude& a)
{
    return std::isfinite(a.radarLat_deg) && std::isfinite(a.radarLon_deg) &&
           std::isfinite(a.radarAlt_km) && std::isfinite(a.roll_deg) &&
           std::isfinite(a.pitch_deg) && std::isfinite(a.yaw_deg);
}

bool IsRotationMatrix(const math::Matrix& m, double tol)
{
    if (m.Rows() != 3 || m.Columns() != 3)
    {
        return false;
    }

    for (std::size_t i = 0; i < 3; ++i)
    {
        for (std::size_t j = 0; j < 3; ++j)
        {
            if (!std::isfinite(m(i, j)))
            {
                return false;
            }
        }
    }

    // R^T * R = I
    const math::Matrix rtr = m.Transpose() * m;
    for (std::size_t i = 0; i < 3; ++i)
    {
        for (std::size_t j = 0; j < 3; ++j)
        {
            if (std::fabs(rtr(i, j) - (i == j ? 1.0 : 0.0)) > tol)
            {
                return false;
            }
        }
    }

    const double det = m(0, 0) * (m(1, 1) * m(2, 2) - m(1, 2) * m(2, 1)) -
                       m(0, 1) * (m(1, 0) * m(2, 2) - m(1, 2) * m(2, 0)) +
                       m(0, 2) * (m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0));
    return std::fabs(det - 1.0) <= tol;
}

double Clamp1(double x)
{
    return std::fmax(-1.0, std::fmin(1.0, x));
}

double Wrap180(double d)
{
    d = std::fmod(d + 180.0, 360.0);
    if (d < 0.0)
    {
        d += 360.0;
    }
    return d - 180.0;
}

math::Matrix AedToVec(double az_deg, double el_deg)
{
    const double az = math::DegToRad(az_deg);
    const double el = math::DegToRad(el_deg);
    return math::Matrix{
        {std::cos(el) * std::sin(az)}, {std::cos(el) * std::cos(az)}, {std::sin(el)}};
}

void VecToAed(const math::Matrix& v, double& az_deg, double& el_deg)
{
    az_deg = Wrap180(math::RadToDeg(std::atan2(v(0, 0), v(1, 0))));
    el_deg = math::RadToDeg(std::asin(Clamp1(v(2, 0))));
}
} // namespace

AntennaToEnu::AntennaToEnu(const RadarAttitude& a, const AttitudeConfig& cfg) : _attitude(a)
{
    if (!AllFinite(a))
    {
        throw AttitudeError(AttitudeErrorCode::NotFinite, "RadarAttitude contains NaN or Inf");
    }

    if (a.radarLat_deg < -90 || a.radarLat_deg > 90 || a.radarLon_deg < -180 ||
        a.radarLon_deg > 180 || a.radarAlt_km < -0.5 || a.radarAlt_km > 100 || a.roll_deg < -180 ||
        a.roll_deg > 180 || a.yaw_deg < -360 || a.yaw_deg > 360)
    {
        throw AttitudeError(AttitudeErrorCode::OutOfRange, "RadarAttitude value is out of range");
    }

    if (std::fabs(a.pitch_deg) >= cfg.maxAbsPitch_deg)
    {
        throw AttitudeError(AttitudeErrorCode::GimbalLock, "pitch is too close to gimbal lock");
    }

    if (!IsRotationMatrix(cfg.mountAntToBody, cfg.mountOrthoTol))
    {
        throw AttitudeError(AttitudeErrorCode::InvalidMount,
                            "mount_ant_to_body is not a rotation matrix");
    }

    const math::Matrix rotationMatrix =
        BodyToEnu(a.roll_deg, a.pitch_deg, a.yaw_deg) * cfg.mountAntToBody;
    _rotAntToEnu = rotationMatrix;
}

void AntennaToEnu::AntToEnuAngle(double azAnt_deg, double elAnt_deg, double& azEnu_deg,
                                 double& elEnu_deg) const
{
    const math::Matrix enu = _rotAntToEnu * AedToVec(azAnt_deg, elAnt_deg);
    VecToAed(enu, azEnu_deg, elEnu_deg);
}

void AntennaToEnu::EnuToAntAngle(double azEnu_deg, double elEnu_deg, double& azAnt_deg,
                                 double& elAnt_deg) const
{
    const math::Matrix antenna = _rotAntToEnu.Transpose() * AedToVec(azEnu_deg, elEnu_deg);
    VecToAed(antenna, azAnt_deg, elAnt_deg);
}

} // namespace beam
