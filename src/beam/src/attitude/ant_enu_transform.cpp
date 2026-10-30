#include "attitude/ant_enu_transform.hpp"

#include <math/quantity/angle.hpp>
#include <math/geometry/spherical_direction.hpp>
#include <math/linear_algebra/square_matrix.hpp>
#include <math/trigonometry.hpp>
#include <math/linear_algebra/vector.hpp>

#include <cmath>
#include <stdexcept>

namespace beam
{
namespace
{
// 몸체 좌표계(x: 오른쪽, y: 정면, z: 위)의 벡터를 ENU(x: 동, y: 북, z: 위)로 바꾸는 회전 행렬을 만든다.
// roll(y축) → pitch(x축) → yaw(z축) 순서로 회전하므로 R = yawRotation * pitchRotation * rollRotation이다.
// 부호: roll은 오른쪽이 아래로 기울 때, pitch는 정면이 위로 들릴 때, yaw는 정면이 북쪽에서 동쪽(시계 방향)으로 돌 때 양수다.
math::SquareMatrix bodyToEnu(math::CircularAngle roll, math::TiltAngle pitch, math::CircularAngle yaw)
{
    const double cr = math::cos(roll);
    const double sr = math::sin(roll);
    const double cp = math::cos(pitch);
    const double sp = math::sin(pitch);
    const double cy = math::cos(yaw);
    const double sy = math::sin(yaw);

    const math::SquareMatrix yawRotation{{cy, sy, 0}, {-sy, cy, 0}, {0, 0, 1}};
    const math::SquareMatrix pitchRotation{{1, 0, 0}, {0, cp, -sp}, {0, sp, cp}};
    const math::SquareMatrix rollRotation{{cr, 0, sr}, {0, 1, 0}, {-sr, 0, cr}};

    return yawRotation * (pitchRotation * rollRotation);
}

} // namespace

AntEnuTransform::AntEnuTransform(const RadarAttitude& attitude, const AttitudeConfig& config)
{
    const auto pitch = math::Angle::fromDegrees(attitude.pitch().deg());
    if (pitch <= -config.maxAbsPitch() || pitch >= config.maxAbsPitch())
    {
        throw std::invalid_argument("AntEnuTransform: pitch is too close to gimbal lock");
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
