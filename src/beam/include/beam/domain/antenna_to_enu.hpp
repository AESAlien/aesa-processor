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

    void AntToEnuAngle(double azAnt_deg, double elAnt_deg,
                       double& azEnu_deg, double& elEnu_deg) const;

    void EnuToAntAngle(double azEnu_deg, double elEnu_deg,
                       double& azAnt_deg, double& elAnt_deg) const;

private:
    RadarAttitude _attitude{};
    math::Matrix _rotAntToEnu;
};

} // namespace beam
