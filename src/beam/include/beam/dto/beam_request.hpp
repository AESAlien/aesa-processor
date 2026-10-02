#pragma once
#include <beam/domain/beam_status.hpp>
#include <cstdint>

namespace beam
{

struct BeamRequest
{
    BeamType beamType{};
    std::uint32_t timestamp_ms{};
    float azimuth_deg{};
    float elevation_deg{};
};

static_assert(sizeof(BeamRequest) == 16, "BeamRequest must be 16 bytes");

} // namespace beam
