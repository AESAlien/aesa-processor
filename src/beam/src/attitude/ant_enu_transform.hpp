#pragma once

#include <beam/config/attitude_config.hpp>
#include "attitude/radar_attitude.hpp"
#include <math/quantity/angle.hpp>
#include <math/geometry/spherical_direction.hpp>
#include <math/linear_algebra/square_matrix.hpp>

namespace aesa_processor::beam
{

class AntEnuTransform
{
public:
    AntEnuTransform(const RadarAttitude&, const AttitudeConfig&);

    aesa_processor::math::SphericalDirection antToEnu(const aesa_processor::math::SphericalDirection& direction_ant) const;

    const aesa_processor::math::SquareMatrix& rotationMatrix() const;

    aesa_processor::math::SphericalDirection enuToAnt(const aesa_processor::math::SphericalDirection& direction_enu) const;

private:
    aesa_processor::math::SquareMatrix _rotationMatrix;
};

} // namespace aesa_processor::beam
