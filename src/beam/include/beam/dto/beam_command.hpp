#pragma once
#include <beam/domain/beam_status.hpp>
#include <cstdint>

namespace beam
{

struct BeamCommand {
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
