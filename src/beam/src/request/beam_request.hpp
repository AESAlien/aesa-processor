#pragma once

#include <beam/dto/beam_type.hpp>
#include <beam/domain/beam_width.hpp>

#include <math/geometry/spherical_direction.hpp>
#include <chrono>
#include <cstdint>

namespace beam
{

// beam 내부에서 쓰는 확인/추적 빔 요청. 생성된 이후에는 항상 유효한 값을 가진다.
// 종류별 값(송신 빔 종류, 빔 폭, 우선순위)은 BeamRequestFactory가 요청 빔 종류에 맞게 정해서 넘기므로
// BeamScheduler와 RequestQueue는 요청의 종류를 알 필요가 없다.
class BeamRequest
{
public:
    // 다음 중 하나라도 해당하면 std::invalid_argument를 던진다.
    // - transmitTime이 음수
    // - requestId가 0
    BeamRequest(
        BeamType transmitBeamType,
        BeamWidth beamWidth,
        int priority,
        std::chrono::milliseconds transmitTime,
        std::uint32_t requestId,
        math::SphericalDirection direction_enu);

    // 송신 시 명령에 담는 빔 종류.
    BeamType transmitBeamType() const noexcept;

    // 송신 빔의 서로 독립적인 방위각/고각 빔 폭.
    BeamWidth beamWidth() const noexcept;

    // 송신 시각이 같은 요청 중 어느 것을 먼저 보낼지 정한다. 값이 클수록 먼저 보낸다.
    int priority() const noexcept;

    std::chrono::milliseconds transmitTime() const noexcept;
    std::uint32_t requestId() const noexcept;
    const math::SphericalDirection& direction_enu() const noexcept;

private:
    void validate() const;

    BeamType _transmitBeamType;
    BeamWidth _beamWidth;
    int _priority;
    std::chrono::milliseconds _transmitTime;
    std::uint32_t _requestId;
    math::SphericalDirection _direction_enu;
};

} // namespace beam
