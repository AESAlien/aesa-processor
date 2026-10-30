#pragma once

#include "scheduler/beam_request.hpp"

#include <beam/config/request_beam_config.hpp>
#include <beam/dto/request_beam_command.hpp>

namespace beam
{

// 공개 DTO인 RequestBeamCommand를 내부 요청(BeamRequest)으로 바꾸는 유일한 곳이다.
// 요청 빔 종류에 따른 분기(송신 빔 종류, 빔 폭, 우선순위)는 여기에만 두므로,
// 사용하는 쪽(BeamController)과 BeamScheduler는 종류별 분기를 알 필요가 없다.
class BeamRequestFactory
{
public:
    // 설정의 빔 폭은 만드는 요청에 반영된다. 설정은 RequestBeamConfig 생성 시 이미 검증되어 있다.
    explicit BeamRequestFactory(const RequestBeamConfig& config);

    // 다음 중 하나라도 해당하면 std::invalid_argument를 던진다.
    // - beamType이 RequestBeamType의 값이 아님
    // - transmitTime이 음수
    // - requestId가 0
    BeamRequest create(const RequestBeamCommand& command) const;

private:
    RequestBeamConfig _config;
};

} // namespace beam
