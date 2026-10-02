#pragma once

#include "attitude/attitude_config.hpp"
#include "attitude/radar_attitude.hpp"
#include <math/matrix.hpp>
#include <tuple>

namespace beam
{

class AntEnuTransform
{
public:
    AntEnuTransform(const RadarAttitude&, const AttitudeConfig&);

    // return (azimuth_enu_deg, elevation_enu_deg)
    std::tuple<double, double> antToEnu(double azimuth_ant_deg, double elevation_ant_deg) const;

    // return (azimuth_ant_deg, elevation_ant_deg)
    std::tuple<double, double> enuToAnt(double azimuth_enu_deg, double elevation_enu_deg) const;

private:
    RadarAttitude _attitude{};
    math::Matrix _rotationMatrix;
};

} // namespace beam
