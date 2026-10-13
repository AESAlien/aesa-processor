#pragma once

#include <beam/config/beam_config.hpp>
#include "attitude/radar_attitude.hpp"
#include <math/quantity/angle.hpp>
#include <math/geometry/spherical_direction.hpp>
#include <math/linear_algebra/square_matrix.hpp>

namespace beam
{

class AntEnuTransform
{
public:
    AntEnuTransform(const RadarAttitude&, const AttitudeConfig&);

    math::SphericalDirection antToEnu(const math::SphericalDirection& direction_ant) const;

    const math::SquareMatrix& rotationMatrix() const;

    math::SphericalDirection enuToAnt(const math::SphericalDirection& direction_enu) const;

private:
    RadarAttitude _attitude{};
    math::SquareMatrix _rotationMatrix;
};

} // namespace beam
