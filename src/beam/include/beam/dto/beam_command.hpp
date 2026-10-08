#pragma once

#include <math/angle.hpp>
#include <chrono>
#include <cstdint>
#include <optional>

namespace beam
{

struct BeamCommand
{
    enum class BeamType : uint8_t;

    const BeamType beamType;
    const std::chrono::milliseconds transmitTime;
    const uint32_t beamId;
    const uint32_t commandCount;
    const math::Angle azimuth_ant;
    const math::Angle elevation_ant;
    const math::Angle azimuthBeamWidth;
    const math::Angle elevationBeamWidth;

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

    class Builder
    {
    public:
        Builder& beamType(BeamType value) { _beamType = value; return *this; }
        Builder& transmitTime(std::chrono::milliseconds value) { _transmitTime = value; return *this; }
        Builder& beamId(uint32_t value) { _beamId = value; return *this; }
        Builder& commandCount(uint32_t value) { _commandCount = value; return *this; }
        Builder& azimuth_ant(math::Angle value) { _azimuth_ant = value; return *this; }
        Builder& elevation_ant(math::Angle value) { _elevation_ant = value; return *this; }
        Builder& azimuthBeamWidth(math::Angle value) { _azimuthBeamWidth = value; return *this; }
        Builder& elevationBeamWidth(math::Angle value) { _elevationBeamWidth = value; return *this; }

        BeamCommand build() const
        {
            return BeamCommand(
                _beamType.value(),
                _transmitTime.value(),
                _beamId.value(),
                _commandCount.value(),
                _azimuth_ant.value(),
                _elevation_ant.value(),
                _azimuthBeamWidth.value(),
                _elevationBeamWidth.value());
        }

    private:
        std::optional<BeamType> _beamType;
        std::optional<std::chrono::milliseconds> _transmitTime;
        std::optional<uint32_t> _beamId;
        std::optional<uint32_t> _commandCount;
        std::optional<math::Angle> _azimuth_ant;
        std::optional<math::Angle> _elevation_ant;
        std::optional<math::Angle> _azimuthBeamWidth;
        std::optional<math::Angle> _elevationBeamWidth;
    };
};

} // namespace beam
