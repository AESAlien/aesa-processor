#pragma once
#include <beam/domain/beam_status.hpp>
#include <cstdint>

namespace beam
{

struct BeamRequest {
    beam::BeamType    beamType{};
    std::uint32_t   timestamp_ms{};
    std::uint32_t   beamID{};
    std::uint32_t   commandCount{};
    float   beam_az_deg{};
    float   beam_el_deg{};
    float   az_width_deg{};
    float   el_width_deg{};
};

static_assert(sizeof(BeamRequest) == 32, "BeamRequest must be 32 bytes");

}   // namespace beam
