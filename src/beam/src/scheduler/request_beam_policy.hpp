#pragma once

#include <beam/dto/beam_type.hpp>

#include <math/quantity/angle.hpp>

namespace beam
{

// 요청 빔(확인/추적)의 종류별 동작을 담은 정책이다. 빔 폭 같은 설정 값은 정책을 만들 때 주입되므로 정책은 상태를 바꾸지 않는다.
// 정책을 사용하는 쪽(BeamRequest)은 이 인터페이스만 보고, 정책을 만드는 일과 종류에 따른 분기는
// RequestBeamPolicyFactory(request_beam_policy_factory.hpp)가 맡는다.
class IRequestBeamPolicy
{
public:
    virtual ~IRequestBeamPolicy() = default;

    // 송신 시 명령에 담는 빔 종류.
    virtual BeamType transmitBeamType() const = 0;

    // 송신 빔의 방위각/고각 빔 폭.
    virtual math::Angle beamWidth() const = 0;

    // 송신 시각이 같은 요청 중 어느 것을 먼저 보낼지 정한다. 값이 클수록 먼저 보낸다.
    virtual int priority() const = 0;
};

} // namespace beam
