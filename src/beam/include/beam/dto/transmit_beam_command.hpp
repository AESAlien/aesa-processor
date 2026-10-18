#pragma once

#include <beam/dto/beam_type.hpp>

#include <math/geometry/spherical_direction.hpp>
#include <math/quantity/angle.hpp>
#include <chrono>
#include <cstdint>
#include <optional>

namespace beam
{

// 시나리오 모의기로 송신할 빔 명령.
// direction_ant는 안테나 좌표계 빔 지향 방향이다.
struct TransmitBeamCommand
{
    const BeamType beamType;
    const std::chrono::milliseconds transmitTime;
    const uint32_t beamId;
    const uint32_t commandCount;
    const math::SphericalDirection direction_ant;
    const math::Angle azimuthBeamWidth;
    const math::Angle elevationBeamWidth;

    TransmitBeamCommand(
        BeamType beamType,
        std::chrono::milliseconds transmitTime,
        uint32_t beamId,
        uint32_t commandCount,
        math::SphericalDirection direction_ant,
        math::Angle azimuthBeamWidth,
        math::Angle elevationBeamWidth
    ) : beamType(beamType),
        transmitTime(transmitTime),
        beamId(beamId),
        commandCount(commandCount),
        direction_ant(direction_ant),
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
        // SphericalDirection은 대입할 수 없으므로 emplace로 다시 만든다.
        Builder& direction_ant(math::SphericalDirection value) { _direction_ant.emplace(value); return *this; }
        Builder& azimuthBeamWidth(math::Angle value) { _azimuthBeamWidth = value; return *this; }
        Builder& elevationBeamWidth(math::Angle value) { _elevationBeamWidth = value; return *this; }

        TransmitBeamCommand build() const
        {
            return TransmitBeamCommand(
                _beamType.value(),
                _transmitTime.value(),
                _beamId.value(),
                _commandCount.value(),
                _direction_ant.value(),
                _azimuthBeamWidth.value(),
                _elevationBeamWidth.value());
        }

    private:
        std::optional<BeamType> _beamType;
        std::optional<std::chrono::milliseconds> _transmitTime;
        std::optional<uint32_t> _beamId;
        std::optional<uint32_t> _commandCount;
        std::optional<math::SphericalDirection> _direction_ant;
        std::optional<math::Angle> _azimuthBeamWidth;
        std::optional<math::Angle> _elevationBeamWidth;
    };
};

} // namespace beam
