#pragma once

#include <cstdint>

namespace beam
{

// 확인/추적 빔 요청의 처리 상태.
enum class RequestBeamStatus : uint8_t
{
    // 큐에 넣었다. 송신까지 끝났다는 뜻은 아니다.
    ACCEPTED,
    // 준비 상태(ON이고 자세정보를 받음)가 아니어서 버렸다.
    NOT_READY,
    // 요청 방향이 추적 영역(안테나 기준) 밖이어서 버렸다.
    OUT_OF_COVERAGE
};

// BeamController::requestBeam의 처리 결과. 요청한 쪽(target)이 거부된 요청에 대응할 수 있도록 반환한다.
// - requestId는 받은 RequestBeamCommand의 값을 그대로 담는다. target은 이 값(표적 ID)으로 대상 표적을 찾는다.
//   표적 ID는 재사용되지 않고 표적마다 대기 중인 요청은 하나뿐이므로, 빔 종류나 송신 시각 없이도 대상을 정할 수 있다.
// - 값을 검사하지 않는다. 검증은 BeamController::requestBeam에서 결과를 만들기 전에 끝나 있다.
struct RequestBeamResult
{
    RequestBeamStatus status;
    uint32_t requestId;

    RequestBeamResult(
        RequestBeamStatus status,
        uint32_t requestId
    ) : status(status),
        requestId(requestId)
    {
    }
};

} // namespace beam
