#pragma once
#include <beam/dto/radar_attitude.hpp>
#include <beam/dto/antenna_to_enu.hpp>
#include <cstdint>
#include <array>

namespace beam
{

struct AttitudeConfig
{
    double max_abs_pitch_deg = 89.0;
    std::array<double,9> mount_ant_to_body{1,0,0, 0,1,0, 0,0,1};
    double mount_ortho_tol = 1e-6;
};

enum class AttitudeCheckResult : std::uint8_t { Ok, NotFinite, OutOfRange, GimbalLock, InvalidMount };

AttitudeCheckResult makeAntennaToEnu(const RadarAttitude&, const AttitudeConfig&, AntennaToEnu& out);

}   // namespace beam