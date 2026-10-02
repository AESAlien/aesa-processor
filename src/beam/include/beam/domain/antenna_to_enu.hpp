#pragma once
#include <beam/domain/radar_attitude.hpp>
#include <math/matrix.hpp>
#include <cstdint>

namespace beam
{

struct AntennaToEnu
{
    RadarAttitude att{};
    math::Matrix rot_ant_to_enu = math::Matrix(3, 3);
    bool valid = false;
};

}   // namespace beam