#pragma once
#include <beam/domain/beam_status.hpp>
#include <cstdint>

namespace dto
{

struct BeamDto {
    beam::BeamType    beamType;
    std::uint32_t   timestamp;    // msec
    std::uint32_t   beamID;
    std::uint32_t   commandCount;
    float   beam_az_deg;
    float   beam_el_deg;
    float   az_width_deg;
    float   el_width_deg;
};

static_assert(sizeof(BeamDto) == 32, "BeamDto must be 32 bytes");

}
