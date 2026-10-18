#pragma once

#include <beam/dto/beam_type.hpp>
#include <beam/dto/request_beam_command.hpp>

#include <math/geometry/spherical_direction.hpp>
#include <chrono>
#include <cstdint>

namespace beam
{

// beam 내부에서 쓰는 확인/추적 빔 요청. 생성된 이후에는 항상 유효한 값을 가진다.
// 공개 DTO인 RequestBeamCommand를 BeamController가 이 클래스로 변환하며, Scheduler와 RequestQueue는 이 클래스만 쓴다.
class BeamRequest
{
public:
    // 다음 중 하나라도 해당하면 std::invalid_argument를 던진다.
    // - beamType이 RequestBeamType의 값이 아님
    // - transmitTime이 음수
    // - requestId가 0
    BeamRequest(
        RequestBeamType beamType,
        std::chrono::milliseconds transmitTime,
        std::uint32_t requestId,
        math::SphericalDirection direction_ant);

    // command의 값으로 만든다. 검증은 위 생성자와 같다.
    explicit BeamRequest(const RequestBeamCommand& command);

    RequestBeamType beamType() const noexcept;
    std::chrono::milliseconds transmitTime() const noexcept;
    std::uint32_t requestId() const noexcept;
    const math::SphericalDirection& direction_ant() const noexcept;

private:
    RequestBeamType _beamType;
    std::chrono::milliseconds _transmitTime;
    std::uint32_t _requestId;
    math::SphericalDirection _direction_ant;
};

} // namespace beam
