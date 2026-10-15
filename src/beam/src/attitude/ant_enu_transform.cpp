#include "attitude/ant_enu_transform.hpp"
#include "attitude/attitude_error.hpp"

#include <cmath>
#include <math/quantity/angle.hpp>
#include <math/geometry/spherical_direction.hpp>
#include <math/linear_algebra/square_matrix.hpp>
#include <math/trigonometry.hpp>
#include <math/linear_algebra/vector.hpp>

namespace beam
{
namespace
{
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

} // namespace

AntEnuTransform::AntEnuTransform(const RadarAttitude& attitude, const AttitudeConfig& config)
{
    if (attitude.pitch() <= -config.maxAbsPitch() || attitude.pitch() >= config.maxAbsPitch())
    {
        throw AttitudeError(AttitudeErrorCode::GIMBAL_LOCK, "pitch is too close to gimbal lock");
    }

    _rotationMatrix =
        bodyToEnu(attitude.roll(), attitude.pitch(), attitude.yaw()) * config.mountAntToBodyRotation();
}

math::SphericalDirection AntEnuTransform::antToEnu(const math::SphericalDirection& direction_ant) const
{
    const math::Vector vector_ant = direction_ant.toCartesianUnitVector();
    const math::Vector vector_enu = _rotationMatrix * vector_ant;
    return math::SphericalDirection::fromCartesianUnitVector(vector_enu);
}

const math::SquareMatrix& AntEnuTransform::rotationMatrix() const
{
    return _rotationMatrix;
}

math::SphericalDirection AntEnuTransform::enuToAnt(const math::SphericalDirection& direction_enu) const
{
    const math::Vector vector_enu = direction_enu.toCartesianUnitVector();
    const math::Vector vector_ant = math::transpose(_rotationMatrix) * vector_enu;
    return math::SphericalDirection::fromCartesianUnitVector(vector_ant);
}

} // namespace beam
