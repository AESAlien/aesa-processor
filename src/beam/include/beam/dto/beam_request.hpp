#pragma once
#include <cstdint>

namespace beam
{

struct BeamRequest
{
    enum class BeamType : std::uint8_t
    {
        CONFIRMATION = 2,
        TRACKING = 3
    };

    BeamType beamType{};
    std::uint32_t timestamp_ms{};
    float azimuth_ant_deg{};
    float elevation_ant_deg{};
};

static_assert(sizeof(BeamRequest) == 16, "BeamRequest must be 16 bytes");

} // namespace beam
