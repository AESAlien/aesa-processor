#include "attitude/ant_enu_transform.hpp"
#include "attitude/attitude_error.hpp"

#include <cmath>
#include <math/quantity/angle.hpp>
#include <math/geometry/spherical_direction.hpp>
#include <math/linear_algebra/square_matrix.hpp>
#include <math/trigonometry.hpp>
#include <math/linear_algebra/vector.hpp>

namespace aesa_processor::beam
{
namespace
{
aesa_processor::math::SquareMatrix bodyToEnu(aesa_processor::math::Angle roll, aesa_processor::math::Angle pitch, aesa_processor::math::Angle yaw)
{
    const double cr = aesa_processor::math::cos(roll);
    const double sr = aesa_processor::math::sin(roll);
    const double cp = aesa_processor::math::cos(pitch);
    const double sp = aesa_processor::math::sin(pitch);
    const double cy = aesa_processor::math::cos(yaw);
    const double sy = aesa_processor::math::sin(yaw);

    const aesa_processor::math::SquareMatrix Rz{{cy, sy, 0}, {-sy, cy, 0}, {0, 0, 1}};
    const aesa_processor::math::SquareMatrix Rx{{1, 0, 0}, {0, cp, -sp}, {0, sp, cp}};
    const aesa_processor::math::SquareMatrix Ry{{cr, 0, sr}, {0, 1, 0}, {-sr, 0, cr}};

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

aesa_processor::math::SphericalDirection AntEnuTransform::antToEnu(const aesa_processor::math::SphericalDirection& direction_ant) const
{
    const aesa_processor::math::Vector vector_ant = direction_ant.toCartesianUnitVector();
    const aesa_processor::math::Vector vector_enu = _rotationMatrix * vector_ant;
    return aesa_processor::math::SphericalDirection::fromCartesianUnitVector(vector_enu);
}

const aesa_processor::math::SquareMatrix& AntEnuTransform::rotationMatrix() const
{
    return _rotationMatrix;
}

aesa_processor::math::SphericalDirection AntEnuTransform::enuToAnt(const aesa_processor::math::SphericalDirection& direction_enu) const
{
    const aesa_processor::math::Vector vector_enu = direction_enu.toCartesianUnitVector();
    const aesa_processor::math::Vector vector_ant = aesa_processor::math::transpose(_rotationMatrix) * vector_enu;
    return aesa_processor::math::SphericalDirection::fromCartesianUnitVector(vector_ant);
}

} // namespace aesa_processor::beam
