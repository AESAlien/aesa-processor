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

bool isRotationMatrix(const math::Matrix& matrix, double tolerance)
{
    if (matrix.rows() != 3 || matrix.columns() != 3 || !std::isfinite(tolerance) || tolerance < 0.0)
    {
        return false;
    }

    if (matrix.isOrthogonal(tolerance) == false)
    {
        return false;
    }

    if (std::fabs(matrix.det() - 1.0) > tolerance)
    {
        return false;
    }

    return true;
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

    if (attitude.latitude < -90_deg || attitude.latitude > 90_deg ||
        attitude.longitude < -180_deg || attitude.longitude > 180_deg ||
        attitude.altitude < -0.5_km || attitude.altitude > 100_km ||
        attitude.roll < -180_deg || attitude.roll > 180_deg ||
        attitude.yaw < -360_deg || attitude.yaw > 360_deg
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
