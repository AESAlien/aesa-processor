#pragma once

#include <math/angle.hpp>
#include <chrono>
#include <cstdint>

namespace beam
{

using math::literals::operator""_deg;
using std::chrono_literals::operator""ms;

struct BeamRequest
{
    enum class BeamType : uint8_t
    {
        CONFIRMATION = 2,
        TRACKING = 3
    };

    BeamType beamType{};
    std::chrono::milliseconds timestamp = 0ms;
    math::Angle azimuth_ant = 0_deg;
    math::Angle elevation_ant = 0_deg;
};

} // namespace beam
