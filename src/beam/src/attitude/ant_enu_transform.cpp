#include "attitude/ant_enu_transform.hpp"
#include "attitude/attitude_error.hpp"

#include <cmath>
#include <math/angle.hpp>
#include <math/matrix.hpp>

namespace beam
{
namespace
{
using namespace math::literals;

math::Matrix bodyToEnu(math::Angle roll, math::Angle pitch, math::Angle yaw)
{
    const double cr = std::cos(roll.rad());
    const double sr = std::sin(roll.rad());
    const double cp = std::cos(pitch.rad());
    const double sp = std::sin(pitch.rad());
    const double cy = std::cos(yaw.rad());
    const double sy = std::sin(yaw.rad());

    const math::Matrix Rz{{cy, sy, 0}, {-sy, cy, 0}, {0, 0, 1}};
    const math::Matrix Rx{{1, 0, 0}, {0, cp, -sp}, {0, sp, cp}};
    const math::Matrix Ry{{cr, 0, sr}, {0, 1, 0}, {-sr, 0, cr}};

    return Rz * (Rx * Ry);
}

bool allFinite(const RadarAttitude& attitude)
{
    return std::isfinite(attitude.latitude.deg()) && std::isfinite(attitude.longitude.deg()) &&
           std::isfinite(attitude.altitude.km()) && std::isfinite(attitude.roll.deg()) &&
           std::isfinite(attitude.pitch.deg()) && std::isfinite(attitude.yaw.deg());
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

math::Matrix aedToVector(math::Angle azimuth, math::Angle elevation)
{
    return math::Matrix{
        {std::cos(elevation.rad()) * std::sin(azimuth.rad())},
        {std::cos(elevation.rad()) * std::cos(azimuth.rad())},
        {std::sin(elevation.rad())}
    };
}

std::tuple<math::Angle, math::Angle> vectorToAed(const math::Matrix& v)
{
    const auto azimuth = math::Angle::fromRadians(std::atan2(v(0, 0), v(1, 0))).wrap180();
    const auto elevation = math::Angle::fromRadians(std::asin(clamp1(v(2, 0))));

    return {azimuth, elevation};
}
} // namespace

AntEnuTransform::AntEnuTransform(const RadarAttitude& attitude, const AttitudeConfig& config) : _attitude(attitude)
{
    if (!allFinite(attitude))
    {
        throw AttitudeError(AttitudeErrorCode::NOT_FINITE, "RadarAttitude contains NaN or Inf");
    }

    if (attitude.latitude  < -90_deg  || attitude.latitude  > 90_deg ||
        attitude.longitude < -180_deg || attitude.longitude > 180_deg ||
        attitude.altitude  < -0.5_km  || attitude.altitude  > 100_km ||
        attitude.roll      < -180_deg || attitude.roll      > 180_deg ||
        attitude.yaw       < -360_deg || attitude.yaw       > 360_deg
    ) {
        throw AttitudeError(AttitudeErrorCode::OUT_OF_RANGE, "RadarAttitude value is out of range");
    }

    if (attitude.pitch <= -config.maxAbsPitch || attitude.pitch >= config.maxAbsPitch)
    {
        throw AttitudeError(AttitudeErrorCode::GIMBAL_LOCK, "pitch is too close to gimbal lock");
    }

    if (!isRotationMatrix(config.mountAntToBodyRotation, config.mountOrthogonalTolerance))
    {
        throw AttitudeError(AttitudeErrorCode::INVALID_MOUNT, "mountAntToBodyRotation is not a rotation matrix");
    }

    _rotationMatrix =
        bodyToEnu(attitude.roll, attitude.pitch, attitude.yaw) * config.mountAntToBodyRotation;
}

std::tuple<math::Angle, math::Angle> AntEnuTransform::antToEnu(math::Angle azimuth_ant, math::Angle elevation_ant) const
{
    const math::Matrix vector_ant = aedToVector(azimuth_ant, elevation_ant);
    const math::Matrix vector_enu = _rotationMatrix * vector_ant;
    return vectorToAed(vector_enu);
}

std::tuple<math::Angle, math::Angle> AntEnuTransform::enuToAnt(math::Angle azimuth_enu, math::Angle elevation_enu) const
{
    const math::Matrix vector_enu = aedToVector(azimuth_enu, elevation_enu);
    const math::Matrix vector_ant = _rotationMatrix.transpose() * vector_enu;
    return vectorToAed(vector_ant);
}

} // namespace beam
