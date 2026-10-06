#pragma once

#include "attitude/attitude_config.hpp"
#include "attitude/radar_attitude.hpp"
#include <math/angle.hpp>
#include <math/square_matrix.hpp>
#include <tuple>

namespace beam
{

class AntEnuTransform
{
public:
    AntEnuTransform(const RadarAttitude&, const AttitudeConfig&);

    // return (azimuth_enu, elevation_enu)
    std::tuple<math::Angle, math::Angle> antToEnu(math::Angle azimuth_ant, math::Angle elevation_ant) const;

    const math::SquareMatrix& rotationMatrix() const;

    // return (azimuth_ant, elevation_ant)
    std::tuple<math::Angle, math::Angle> enuToAnt(math::Angle azimuth_enu, math::Angle elevation_enu) const;

private:
    RadarAttitude _attitude{};
    math::SquareMatrix _rotationMatrix;
};

} // namespace beam
