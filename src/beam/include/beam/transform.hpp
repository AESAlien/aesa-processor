#pragma once
#include <beam/dto/antenna_to_enu.hpp>

namespace beam
{
    
void antToEnuAngle(const AntennaToEnu& xform,
    double az_ant_deg,  double el_ant_deg,
    double& az_enu_deg, double& el_enu_deg
);

void enuToAntAngle(const AntennaToEnu& xform,
    double az_enu_deg,  double el_enu_deg,
    double& az_ant_deg, double& el_ant_deg
);

}   // namespace beam
