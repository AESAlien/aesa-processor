#pragma once

#include "scheduler/beam_request.hpp"
#include "scheduler/search_grid.hpp"
#include "scheduler/request_queue.hpp"

#include <beam/domain/operation_state.hpp>
#include <beam/dto/transmit_beam_command.hpp>

#include <chrono>
#include <cstdint>
#include <optional>

namespace beam
{

// 빔 스케줄링 본체. 타이머나 스레드를 갖지 않고, 호출자(app)가 틱마다 next()를 호출한다.
// 탐색 격자(SearchGrid)는 BeamController가 자세정보와 설정으로 만들어 넘기므로, Scheduler는 자세정보와 설정을 알지 못한다.
//
// 가정(계약):
// - 모든 메서드는 같은 스레드에서 직렬로 호출된다. 스레드 안전하지 않다.
// - next()는 틱(10ms)마다 한 번만 호출된다.
// - 탐색 격자(자세정보)는 ON 이후에 한 번만 도착한다.
// - OFF 후 다시 ON하면 탐색 격자도 다시 도착한다(자세정보는 ON마다 재전송). ON 때 모든 상태를 초기화하기 때문이다.
//
// 상태:
// - OFF에서는 모든 입력을 무시하고 next()는 std::nullopt를 반환한다.
// - OFF에서 ON으로 바뀌는 순간 모든 상태를 초기화한다.
// - ON이고 SearchGrid를 받은 상태에서만 빔을 송신한다.
class Scheduler
{
public:
    void setOperationState(OperationState state);

    // ON이고 SearchGrid가 아직 없을 때만 받아들이고, 그 외에는 무시한다.
    void setSearchGrid(SearchGrid searchGrid);

    // 준비 상태(ON이고 SearchGrid가 있음)가 아니면 무시한다.
    // 요청 값의 검증은 BeamRequest 생성 시 끝나 있다.
    // 요청의 방향은 ENU 기준으로 간주하며, 범위(추적 영역) 검증은 하지 않는다.
    void addRequest(const BeamRequest& request);

    // 준비 상태가 아니면 std::nullopt를 반환한다.
    // 송신할 요청이 있으면 요청 빔을, 없으면 탐색 빔을 반환한다.
    std::optional<TransmitBeamCommand> next(std::chrono::milliseconds currentTime);

private:
    bool isReady() const;
    void reset();

    OperationState _operationState = OperationState::OFF;
    std::optional<SearchGrid> _searchGrid;
    RequestQueue _requestQueue;
    std::uint32_t _commandCount = 0;
};

} // namespace beam
