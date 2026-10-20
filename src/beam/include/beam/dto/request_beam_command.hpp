#pragma once

#include <beam/dto/beam_type.hpp>

#include <math/geometry/spherical_direction.hpp>
#include <chrono>
#include <cstdint>
#include <optional>

namespace beam
{

// 확인/추적 빔 송신 요청. target이 만들어 BeamController::requestBeam으로 전달한다.
// - direction_ant는 이름과 달리 ENU 기준 방향이며, 변환 없이 TransmitBeamCommand로 그대로 전달된다.
//   (필드 이름 변경은 comm/target과 합의 후 진행한다.)
// - 좌표 변환과 추적 영역 범위 검증은 요청을 만드는 쪽의 책임이다.
// - 이 DTO는 값을 검사하지 않는다. transmitTime과 requestId의 검증은 BeamController::requestBeam에서
//   내부 요청(BeamRequest)을 만들 때 한다.
// - requestId는 TransmitBeamCommand::beamId로 전달된다.
struct RequestBeamCommand
{
    const RequestBeamType beamType;
    const std::chrono::milliseconds transmitTime;
    const uint32_t requestId;
    const math::SphericalDirection direction_ant;

    RequestBeamCommand(
        RequestBeamType beamType,
        std::chrono::milliseconds transmitTime,
        uint32_t requestId,
        math::SphericalDirection direction_ant
    ) : beamType(beamType),
        transmitTime(transmitTime),
        requestId(requestId),
        direction_ant(direction_ant)
    {
    }

    class Builder
    {
    public:
        Builder& beamType(RequestBeamType value) { _beamType = value; return *this; }
        Builder& transmitTime(std::chrono::milliseconds value) { _transmitTime = value; return *this; }
        Builder& requestId(uint32_t value) { _requestId = value; return *this; }
        // SphericalDirection은 대입할 수 없으므로 emplace로 다시 만든다.
        Builder& direction_ant(math::SphericalDirection value) { _direction_ant.emplace(value); return *this; }

        RequestBeamCommand build() const
        {
            return RequestBeamCommand(
                _beamType.value(),
                _transmitTime.value(),
                _requestId.value(),
                _direction_ant.value());
        }

    private:
        std::optional<RequestBeamType> _beamType;
        std::optional<std::chrono::milliseconds> _transmitTime;
        std::optional<uint32_t> _requestId;
        std::optional<math::SphericalDirection> _direction_ant;
    };
};

} // namespace beam
