#pragma once

#include <math/angle.hpp>
#include <chrono>
#include <cstdint>

namespace beam
{

using math::literals::operator""_deg;
using std::chrono_literals::operator""ms;

struct BeamCommand
{
    enum class BeamType : std::uint8_t
    {
        SEARCH = 1,
        CONFIRMATION = 2,
        TRACKING = 3
    };

    BeamType beamType{};
    // Elapsed time since the shared start at 0 ms.
    std::chrono::milliseconds timestamp = 0ms;
    std::uint32_t beamId{};
    std::uint32_t commandCount{};
    math::Angle azimuth_ant = 0_deg;
    math::Angle elevation_ant = 0_deg;
    math::Angle azimuthBeamWidth = 0_deg;
    math::Angle elevationBeamWidth = 0_deg;
};

} // namespace beam
