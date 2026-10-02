#pragma once

#include <beam/config/attitude_config.hpp>
#include <beam/domain/radar_attitude.hpp>
#include <math/matrix.hpp>

namespace beam
{

class AntennaToEnu
{
public:
    AntennaToEnu(const RadarAttitude&, const AttitudeConfig&);

    void antToEnuAngle(
        double az_ant_deg,
        double el_ant_deg,
        double& az_enu_deg,
        double& el_enu_deg) const;

    void enuToAntAngle(
        double az_enu_deg,
        double el_enu_deg,
        double& az_ant_deg,
        double& el_ant_deg) const;

private:
    RadarAttitude attitude_{};
    math::Matrix rot_ant_to_enu_;
};

}   // namespace beam
