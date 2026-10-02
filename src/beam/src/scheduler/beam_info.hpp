#pragma once
#include <cstdint>

namespace beam
{

struct BeamInfo
{
    std::uint32_t beamID;
    float   az_deg{};
    float   el_deg{};
    float   az_width_deg{};
    float   el_width_deg{};
};

}   // namespace beam
