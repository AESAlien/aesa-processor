#pragma once

#include <math/angle.hpp>
#include <cstdint>

namespace beam
{

using math::literals::operator""_deg;

struct BeamInfo
{
    std::uint32_t beamId;
    math::Angle azimuth_ant = 0_deg;
    math::Angle elevation_ant = 0_deg;
    math::Angle azimuthBeamWidth = 0_deg;
    math::Angle elevationBeamWidth = 0_deg;
};

} // namespace beam
