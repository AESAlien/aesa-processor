#pragma once

#include <pose/domain/radar_attitude.hpp>
#include <pose/config/ant_enu_transform_config.hpp>
#include <math/geometry/spherical_direction.hpp>
#include <math/linear_algebra/square_matrix.hpp>
#include <math/linear_algebra/vector.hpp>

namespace pose
{

class AntEnuTransform
{
public:
    AntEnuTransform(const RadarAttitude& attitude, const AntEnuTransformConfig& config);

    math::SphericalDirection antToEnu(const math::SphericalDirection& direction_ant) const;

    math::Vector antToEnu(const math::Vector& vector_ant) const;

    math::SphericalDirection enuToAnt(const math::SphericalDirection& direction_enu) const;

private:
    math::SquareMatrix _rotationMatrix;
};

} // namespace pose
