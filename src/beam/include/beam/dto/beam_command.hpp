#pragma once
#include <cstdint>

namespace beam
{

struct BeamCommand
{
    enum class BeamType : std::uint8_t
    {
        Search = 1,
        Confirmation = 2,
        Tracking = 3
    };

    BeamType beamType{};
    std::uint32_t timestamp_ms{};
    std::uint32_t beamId{};
};
    BeamType beamType{};
    std::uint32_t timestamp_ms{};
    std::uint32_t beamId{};
    std::uint32_t commandCount{};
    float azimuth_deg{};
    float elevation_deg{};
    float azimuthBeamWidth_deg{};
    float elevationBeamWidth_deg{};
};

static_assert(sizeof(BeamCommand) == 32, "BeamCommand must be 32 bytes");

}   // namespace beam
