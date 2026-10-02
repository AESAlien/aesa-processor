#pragma once
#include <cstdint>

namespace beam
{

struct BeamInfo
{
    std::uint32_t beamId;
    float azimuth_ant_deg{};
    float elevation_ant_deg{};
    float azimuthBeamWidth_deg{};
    float elevationBeamWidth_deg{};
};

} // namespace beam
