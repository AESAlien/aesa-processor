#pragma once

#include "scheduler/request_beam_policy.hpp"

#include <beam/config/request_beam_config.hpp>
#include <beam/dto/beam_type.hpp>

#include <memory>

namespace beam
{

// 요청 빔 종류에 맞는 정책을 만든다. 확인/추적 빔 종류에 따른 분기(switch/case)는 이 클래스에만 둔다.
// 구체 정책 클래스는 .cpp에 숨겨 두므로, 이 팩토리만 구체 정책을 알고 사용하는 쪽은 IRequestBeamPolicy만 본다.
class RequestBeamPolicyFactory
{
public:
    // 설정 값은 생성 시점에 정책에 복사되어 주입된다. 설정은 RequestBeamConfig 생성 시 이미 검증되어 있다.
    explicit RequestBeamPolicyFactory(const RequestBeamConfig& config);

    // 알 수 없는 종류이면 std::invalid_argument를 던진다.
    // 정책은 바뀌지 않으므로 같은 종류에는 같은 정책 객체를 돌려주고, 요청들이 이를 공유한다.
    std::shared_ptr<const IRequestBeamPolicy> create(RequestBeamType beamType) const;

private:
    std::shared_ptr<const IRequestBeamPolicy> _confirmationPolicy;
    std::shared_ptr<const IRequestBeamPolicy> _trackingPolicy;
};

} // namespace beam
