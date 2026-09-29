#pragma once
#include <cstdint>

namespace dto
{

enum class BeamType : std::uint8_t { Search = 1, Confirm = 2, Track = 3 };

struct ST_BeamTx {
    BeamType    beamType;
    std::uint8_t    padding[3] = {};
    std::uint32_t   timestamp;    // msec
    std::uint32_t   beamID;
    std::uint32_t   commandCount;
    float   beam_az_deg;
    float   beam_el_deg;
    float   az_width_deg;
    float   el_width_deg;
    std::uint8_t    padding_data[28] = {};
};

static_assert(sizeof(ST_BeamTx) == 60, "ST_BeamTx must be 60 bytes");

}
