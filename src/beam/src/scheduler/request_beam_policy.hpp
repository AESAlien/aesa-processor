#pragma once

#include <beam/config/request_beam_config.hpp>
#include <beam/dto/beam_type.hpp>

#include <math/quantity/angle.hpp>

namespace beam
{

// 요청 빔(확인/추적)의 종류별 동작을 정한다. 전략은 상태를 갖지 않는다.
// 종류를 추가할 때는 구현체를 만들고 requestBeamPolicyOf에 등록한다.
class IRequestBeamPolicy
{
public:
    virtual ~IRequestBeamPolicy() = default;

    // 송신 명령에 담는 빔 종류.
    virtual BeamType transmitBeamType() const = 0;

    // 송신 빔의 방위각·고각 빔 폭.
    virtual math::Angle beamWidth(const RequestBeamConfig& config) const = 0;

    // 송신 시각이 같은 요청 중 어느 것을 먼저 보낼지 정한다. 값이 클수록 먼저 보낸다.
    virtual int priority() const = 0;
};

// beamType에 해당하는 전략을 반환한다. 반환한 참조는 프로그램이 끝날 때까지 유효하다.
// beamType이 RequestBeamType의 값이 아니면 std::invalid_argument를 던진다.
const IRequestBeamPolicy& requestBeamPolicyOf(RequestBeamType beamType);

} // namespace beam
