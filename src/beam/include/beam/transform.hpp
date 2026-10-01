#pragma once
#include <beam/dto/attTransform_dto.hpp>

namespace beam
{
    
bool antToEnuAngle(const AttTransformDto& xform,
    double az_ant_deg,  double el_ant_deg,
    double& az_enu_deg, double& el_enu_deg
);

bool enuToAntAngle(const AttTransformDto& xform,
    double az_enu_deg,  double el_enu_deg,
    double& az_ant_deg, double& el_ant_deg
);

}   // namespace beam
