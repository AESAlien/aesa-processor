#pragma once

#include <math/angle.hpp>
#include <chrono>
#include <cstdint>

namespace beam
{

struct BeamRequest
{
    enum class BeamType : uint8_t
    {
        CONFIRMATION = 2,
        TRACKING = 3
    };

    BeamRequest(
        BeamType beamType,
        std::chrono::milliseconds transmitTime,
        math::Angle azimuth_ant,
        math::Angle elevation_ant
    ) : beamType(beamType),
        transmitTime(transmitTime),
        azimuth_ant(azimuth_ant),
        elevation_ant(elevation_ant)
    {
    }

    const BeamType beamType;
    const std::chrono::milliseconds transmitTime;
    const math::Angle azimuth_ant;
    const math::Angle elevation_ant;
};

} // namespace beam
