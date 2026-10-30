#pragma once

#include <beam/dto/beam_type.hpp>

#include <chrono>
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
// - beamType, requestId, transmitTime은 받은 RequestBeamCommand의 값을 변환 없이 그대로 담는다.
//   target은 requestId(표적 ID)와 beamType으로 대상 표적을 찾고, transmitTime으로 오래된 결과를 거를 수 있다.
// - 값을 검사하지 않는다. 검증은 BeamController::requestBeam에서 결과를 만들기 전에 끝나 있다.
struct RequestBeamResult
{
    RequestBeamStatus status;
    RequestBeamType beamType;
    std::chrono::milliseconds transmitTime;
    uint32_t requestId;

    RequestBeamResult(
        RequestBeamStatus status,
        RequestBeamType beamType,
        std::chrono::milliseconds transmitTime,
        uint32_t requestId
    ) : status(status),
        beamType(beamType),
        transmitTime(transmitTime),
        requestId(requestId)
    {
    }
};

} // namespace beam
