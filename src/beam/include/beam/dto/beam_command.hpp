#pragma once

#include <math/angle.hpp>
#include <chrono>
#include <cstdint>

namespace beam
{

struct BeamCommand
{
    enum class BeamType : uint8_t
    {
        SEARCH = 1,
        CONFIRMATION = 2,
        TRACKING = 3
    };

    BeamCommand(
        BeamType beamType,
        std::chrono::milliseconds transmitTime,
        uint32_t beamId,
        uint32_t commandCount,
        math::Angle azimuth_ant,
        math::Angle elevation_ant,
        math::Angle azimuthBeamWidth,
        math::Angle elevationBeamWidth
    ) : beamType(beamType),
        transmitTime(transmitTime),
        beamId(beamId),
        commandCount(commandCount),
        azimuth_ant(azimuth_ant),
        elevation_ant(elevation_ant),
        azimuthBeamWidth(azimuthBeamWidth),
        elevationBeamWidth(elevationBeamWidth)
    {
    }

    const BeamType beamType;
    const std::chrono::milliseconds transmitTime;
    const uint32_t beamId;
    const uint32_t commandCount;
    const math::Angle azimuth_ant;
    const math::Angle elevation_ant;
    const math::Angle azimuthBeamWidth;
    const math::Angle elevationBeamWidth;
};

} // namespace beam
