#pragma once
#include <beam/dto/attitude_dto.hpp>
#include <beam/dto/attTransform_dto.hpp>
#include <beam/port/attitude_source.hpp>
#include <cstdint>
#include <array>

namespace beam
{

struct AttitudeConfig
{
    double max_abs_pitch_deg = 89.0;
    std::array<double,9> mount_ant_to_body{1,0,0, 0,1,0, 0,0,1};
};

enum class AttUpdateResult : std::uint8_t { Ok, NotFinite, OutOfRange, GimbalLock };

AttUpdateResult makeAttTransform(const AttitudeDto&, const AttitudeConfig&, AttTransformDto& out);

}   // namespace beam