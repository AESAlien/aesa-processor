#pragma once
#include <cstdint>

namespace beam
{

struct BeamInfo
{
    std::uint32_t beamId;
    float az_deg{};
    float el_deg{};
    float azWidth_deg{};
    float elWidth_deg{};
};

} // namespace beam
