#include "attitude/ant_enu_transform.hpp"
#include "attitude/attitude_error.hpp"

#include <cmath>
#include <stdexcept>
#include <math/quantity/angle.hpp>
#include <math/geometry/spherical_direction.hpp>
#include <math/linear_algebra/square_matrix.hpp>
#include <math/trigonometry.hpp>
#include <math/linear_algebra/vector.hpp>

namespace beam
{
namespace
{
using namespace math::literals;

math::SquareMatrix bodyToEnu(math::Angle roll, math::Angle pitch, math::Angle yaw)
{
    const double cr = math::cos(roll);
    const double sr = math::sin(roll);
    const double cp = math::cos(pitch);
    const double sp = math::sin(pitch);
    const double cy = math::cos(yaw);
    const double sy = math::sin(yaw);

    const math::SquareMatrix Rz{{cy, sy, 0}, {-sy, cy, 0}, {0, 0, 1}};
    const math::SquareMatrix Rx{{1, 0, 0}, {0, cp, -sp}, {0, sp, cp}};
    const math::SquareMatrix Ry{{cr, 0, sr}, {0, 1, 0}, {-sr, 0, cr}};

    return Rz * (Rx * Ry);
}

bool allFinite(const RadarAttitude& attitude)
{
    return attitude.latitude.isFinite() && attitude.longitude.isFinite() &&
           attitude.altitude.isFinite() && attitude.roll.isFinite() &&
           attitude.pitch.isFinite() && attitude.yaw.isFinite();
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

    if (config.mountAntToBodyRotation.size() != 3 ||
        !config.mountAntToBodyRotation.isRotation(config.mountOrthogonalTolerance))
    {
        throw AttitudeError(AttitudeErrorCode::INVALID_MOUNT, "mountAntToBodyRotation is not a rotation matrix");
    }

    _rotationMatrix =
        bodyToEnu(attitude.roll, attitude.pitch, attitude.yaw) * config.mountAntToBodyRotation;
}

math::SphericalDirection AntEnuTransform::antToEnu(const math::SphericalDirection& direction_ant) const
{
    const math::Vector vector_ant = direction_ant.toEnuVector();
    const math::Vector vector_enu = _rotationMatrix * vector_ant;
    return math::SphericalDirection::fromEnuVector(vector_enu);
}

const math::SquareMatrix& AntEnuTransform::rotationMatrix() const
{
    return _rotationMatrix;
}

math::SphericalDirection AntEnuTransform::enuToAnt(const math::SphericalDirection& direction_enu) const
{
    const math::Vector vector_enu = direction_enu.toEnuVector();
    const math::Vector vector_ant = _rotationMatrix.transpose() * vector_enu;
    return math::SphericalDirection::fromEnuVector(vector_ant);
}

} // namespace beam
