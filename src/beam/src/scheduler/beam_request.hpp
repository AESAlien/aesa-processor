#pragma once

#include "scheduler/request_beam_policy.hpp"

#include <beam/dto/beam_type.hpp>

#include <math/geometry/spherical_direction.hpp>
#include <math/quantity/angle.hpp>
#include <chrono>
#include <cstdint>
#include <memory>

namespace beam
{

// beam 내부에서 쓰는 확인/추적 빔 요청. 생성된 이후에는 항상 유효한 값을 가진다.
// 종류별 동작(송신 빔 종류, 빔 폭, 우선순위)은 주입받은 정책에 맡기므로 Scheduler와 RequestQueue는 요청의 종류를 알 필요가 없다.
// 공개 DTO인 RequestBeamCommand는 BeamRequestFactory가 정책을 골라 이 클래스로 변환한다.
class BeamRequest
{
public:
    // 다음 중 하나라도 해당하면 std::invalid_argument를 던진다.
    // - policy가 null
    // - transmitTime이 음수
    // - requestId가 0
    BeamRequest(
        std::shared_ptr<const IRequestBeamPolicy> policy,
        std::chrono::milliseconds transmitTime,
        std::uint32_t requestId,
        math::SphericalDirection direction_enu);

    // 아래 세 메서드는 정책에 위임한다.
    BeamType transmitBeamType() const;
    math::Angle beamWidth() const;
    int priority() const;

    std::chrono::milliseconds transmitTime() const noexcept;
    std::uint32_t requestId() const noexcept;
    const math::SphericalDirection& direction_enu() const noexcept;

private:
    void validate() const;

    std::shared_ptr<const IRequestBeamPolicy> _policy;
    std::chrono::milliseconds _transmitTime;
    std::uint32_t _requestId;
    math::SphericalDirection _direction_enu;
};

} // namespace beam
