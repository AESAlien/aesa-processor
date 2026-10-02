#include "attitude/ant_enu_transform.hpp"
#include "attitude/attitude_error.hpp"

#include <cmath>
#include <math/angle.hpp>
#include <math/matrix.hpp>

namespace beam
{
namespace
{
math::Matrix bodyToEnu(double roll_deg, double pitch_deg, double yaw_deg)
{
    const double cr = std::cos(math::degToRad(roll_deg));
    const double sr = std::sin(math::degToRad(roll_deg));
    const double cp = std::cos(math::degToRad(pitch_deg));
    const double sp = std::sin(math::degToRad(pitch_deg));
    const double cy = std::cos(math::degToRad(yaw_deg));
    const double sy = std::sin(math::degToRad(yaw_deg));

    const math::Matrix Rz{{cy, sy, 0}, {-sy, cy, 0}, {0, 0, 1}};
    const math::Matrix Rx{{1, 0, 0}, {0, cp, -sp}, {0, sp, cp}};
    const math::Matrix Ry{{cr, 0, sr}, {0, 1, 0}, {-sr, 0, cr}};

    return Rz * (Rx * Ry);
}

bool allFinite(const RadarAttitude& attitude)
{
    return std::isfinite(attitude.latitude_deg) && std::isfinite(attitude.longitude_deg) &&
           std::isfinite(attitude.altitude_km) && std::isfinite(attitude.roll_deg) &&
           std::isfinite(attitude.pitch_deg) && std::isfinite(attitude.yaw_deg);
}

bool isRotationMatrix(const math::Matrix& m, double tolerance)
{
    if (m.rows() != 3 || m.columns() != 3)
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
    const math::Matrix RtR = m.transpose() * m;
    for (std::size_t i = 0; i < 3; ++i)
    {
        for (std::size_t j = 0; j < 3; ++j)
        {
            if (std::fabs(RtR(i, j) - (i == j ? 1.0 : 0.0)) > tolerance)
            {
                return false;
            }
        }
    }

    const double det = m(0, 0) * (m(1, 1) * m(2, 2) - m(1, 2) * m(2, 1)) -
                       m(0, 1) * (m(1, 0) * m(2, 2) - m(1, 2) * m(2, 0)) +
                       m(0, 2) * (m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0));
    return std::fabs(det - 1.0) <= tolerance;
}

double clamp1(double x)
{
    return std::fmax(-1.0, std::fmin(1.0, x));
}

double wrap180(double d)
{
    d = std::fmod(d + 180.0, 360.0);
    if (d < 0.0)
    {
        d += 360.0;
    }
    return d - 180.0;
}

math::Matrix aedToVector(double azimuth_deg, double elevation_deg)
{
    const double azimuth_rad = math::degToRad(azimuth_deg);
    const double elevation_rad = math::degToRad(elevation_deg);

    return math::Matrix{{std::cos(elevation_rad) * std::sin(azimuth_rad)},
                        {std::cos(elevation_rad) * std::cos(azimuth_rad)},
                        {std::sin(elevation_rad)}};
}

std::tuple<double, double> vectorToAed(const math::Matrix& v)
{
    const double azimuth_deg = wrap180(math::radToDeg(std::atan2(v(0, 0), v(1, 0))));
    const double elevation_deg = math::radToDeg(std::asin(clamp1(v(2, 0))));

    return {azimuth_deg, elevation_deg};
}
} // namespace

AntEnuTransform::AntEnuTransform(const RadarAttitude& attitude, const AttitudeConfig& config) : _attitude(attitude)
{
    if (!allFinite(attitude))
    {
        throw AttitudeError(AttitudeErrorCode::NOT_FINITE, "RadarAttitude contains NaN or Inf");
    }

    if (attitude.latitude_deg < -90 || attitude.latitude_deg > 90 || attitude.longitude_deg < -180 ||
        attitude.longitude_deg > 180 || attitude.altitude_km < -0.5 || attitude.altitude_km > 100 ||
        attitude.roll_deg < -180 || attitude.roll_deg > 180 || attitude.yaw_deg < -360 || attitude.yaw_deg > 360)
    {
        throw AttitudeError(AttitudeErrorCode::OUT_OF_RANGE, "RadarAttitude value is out of range");
    }

    if (std::fabs(attitude.pitch_deg) >= config.maxAbsPitch_deg)
    {
        throw AttitudeError(AttitudeErrorCode::GIMBAL_LOCK, "pitch is too close to gimbal lock");
    }

    if (!isRotationMatrix(config.mountAntToBodyRotation, config.mountOrthogonalTolerance))
    {
        throw AttitudeError(AttitudeErrorCode::INVALID_MOUNT, "mountAntToBodyRotation is not a rotation matrix");
    }

    _rotationMatrix =
        bodyToEnu(attitude.roll_deg, attitude.pitch_deg, attitude.yaw_deg) * config.mountAntToBodyRotation;
}

std::tuple<double, double> AntEnuTransform::antToEnu(double azimuth_ant_deg, double elevation_ant_deg) const
{
    const math::Matrix vector_ant = aedToVector(azimuth_ant_deg, elevation_ant_deg);
    const math::Matrix vector_enu = _rotationMatrix * vector_ant;
    return vectorToAed(vector_enu);
}

std::tuple<double, double> AntEnuTransform::enuToAnt(double azimuth_enu_deg, double elevation_enu_deg) const
{
    const math::Matrix vector_enu = aedToVector(azimuth_enu_deg, elevation_enu_deg);
    const math::Matrix vector_ant = _rotationMatrix.transpose() * vector_enu;
    return vectorToAed(vector_ant);
}

} // namespace beam
