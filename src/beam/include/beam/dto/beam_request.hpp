#pragma once

#include <math/angle.hpp>
#include <chrono>
#include <cstdint>
#include <optional>

namespace beam
{

// 확인/추적 빔 송신 요청.
// - azimuth_ant, elevation_ant는 안테나 좌표계 각도이며, 변환 없이 BeamCommand로 그대로 전달된다.
// - 좌표 변환과 추적 영역 범위 검증은 요청을 만드는 쪽의 책임이며, 빔 모듈은 값을 검사하지 않는다.
// - requestId는 BeamCommand::beamId로 전달된다.
struct BeamRequest
{
    enum class BeamType : uint8_t;

    const BeamType beamType;
    const std::chrono::milliseconds transmitTime;
    const uint32_t requestId;
    const math::Angle azimuth_ant;
    const math::Angle elevation_ant;

    enum class BeamType : uint8_t
    {
        CONFIRMATION = 2,
        TRACKING = 3
    };

    BeamRequest(
        BeamType beamType,
        std::chrono::milliseconds transmitTime,
        uint32_t requestId,
        math::Angle azimuth_ant,
        math::Angle elevation_ant
    ) : beamType(beamType),
        transmitTime(transmitTime),
        requestId(requestId),
        azimuth_ant(azimuth_ant),
        elevation_ant(elevation_ant)
    {
    }

    class Builder
    {
    public:
        Builder& beamType(BeamType value) { _beamType = value; return *this; }
        Builder& transmitTime(std::chrono::milliseconds value) { _transmitTime = value; return *this; }
        Builder& requestId(uint32_t value) { _requestId = value; return *this; }
        Builder& azimuth_ant(math::Angle value) { _azimuth_ant = value; return *this; }
        Builder& elevation_ant(math::Angle value) { _elevation_ant = value; return *this; }

        BeamRequest build() const
        {
            return BeamRequest(
                _beamType.value(),
                _transmitTime.value(),
                _requestId.value(),
                _azimuth_ant.value(),
                _elevation_ant.value());
        }

    private:
        std::optional<BeamType> _beamType;
        std::optional<std::chrono::milliseconds> _transmitTime;
        std::optional<uint32_t> _requestId;
        std::optional<math::Angle> _azimuth_ant;
        std::optional<math::Angle> _elevation_ant;
    };
};

} // namespace beam
