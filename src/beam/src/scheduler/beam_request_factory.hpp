#pragma once

#include "scheduler/beam_request.hpp"
#include "scheduler/request_beam_policy_factory.hpp"

#include <beam/config/request_beam_config.hpp>
#include <beam/dto/request_beam_command.hpp>

namespace beam
{

// 공개 DTO인 RequestBeamCommand를 내부 요청(BeamRequest)으로 바꾸는 유일한 곳이다.
// 요청 빔 종류에 맞는 정책을 고르고(RequestBeamPolicyFactory) BeamRequest에 주입하는 일을 맡으므로,
// 사용하는 쪽(BeamController)은 정책이나 종류별 분기를 알 필요가 없다.
class BeamRequestFactory
{
public:
    // 설정은 정책에 주입되어 요청의 빔 폭에 반영된다.
    explicit BeamRequestFactory(const RequestBeamConfig& config);

    // 다음 중 하나라도 해당하면 std::invalid_argument를 던진다.
    // - beamType이 RequestBeamType의 값이 아님
    // - transmitTime이 음수
    // - requestId가 0
    BeamRequest create(const RequestBeamCommand& command) const;

private:
    RequestBeamPolicyFactory _policyFactory;
};

} // namespace beam
