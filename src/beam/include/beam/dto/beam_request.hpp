#pragma once
#include <cstdint>

namespace beam
{

struct BeamRequest
{
    enum class BeamType : std::uint8_t
    {
        Confirmation = 2,
        Tracking = 3
    };

    BeamType beamType{};
    std::uint32_t timestamp_ms{};
    float azimuth_deg{};
};
    BeamType beamType{};
    std::uint32_t timestamp_ms{};
    float azimuth_deg{};
    float elevation_deg{};
};

static_assert(sizeof(BeamRequest) == 16, "BeamRequest must be 16 bytes");

}   // namespace beam
