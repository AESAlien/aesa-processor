#pragma once

#include "attitude/radar_attitude.hpp"
#include "scheduler/beam_request.hpp"
#include "scheduler/search_beam_table.hpp"
#include "scheduler/request_queue.hpp"

#include <beam/config/beam_config.hpp>
#include <beam/domain/operation_state.hpp>
#include <beam/dto/transmit_beam_command.hpp>

#include <chrono>
#include <cstdint>
#include <optional>

namespace beam
{

// 빔 스케줄링 본체. 타이머나 스레드를 갖지 않고, 호출자(app)가 틱마다 next()를 호출한다.
//
// 가정(계약):
// - 모든 메서드는 같은 스레드에서 직렬로 호출된다. 스레드 안전하지 않다.
// - next()는 틱(10ms)마다 한 번만 호출된다.
// - 자세정보는 ON 이후에 한 번만 도착한다.
// - OFF 후 다시 ON하면 자세정보도 다시 도착한다(ON마다 재전송). ON 때 모든 상태를 초기화하기 때문이다.
//
// 상태:
// - OFF에서는 모든 입력을 무시하고 next()는 std::nullopt를 반환한다.
// - OFF에서 ON으로 바뀌는 순간 모든 상태를 초기화한다.
// - ON이고 자세정보를 받아 SearchBeamTable이 만들어진 상태에서만 빔을 송신한다.
class Scheduler
{
public:
    explicit Scheduler(const BeamConfig& config = BeamConfig{});

    void setOperationState(OperationState state);

    // 잘못된 자세정보(짐벌락 포함)면 운용 상태와 관계없이 AttitudeError를 던지며, 이때 상태는 변하지 않는다.
    // 올바른 자세정보는 ON이고 SearchBeamTable이 아직 없을 때만 받아들이고, 그 외에는 무시한다.
    void setAttitude(const RadarAttitude& attitude);

    // 준비 상태(ON이고 SearchBeamTable이 있음)가 아니면 무시한다.
    // 요청 값의 검증은 BeamRequest 생성 시 끝나 있다.
    // 요청의 방향은 안테나 좌표계로 간주하며, 범위(추적 영역) 검증은 하지 않는다.
    void addRequest(const BeamRequest& request);

    // 준비 상태가 아니면 std::nullopt를 반환한다.
    // 송신할 요청이 있으면 요청 빔을, 없으면 탐색 빔을 반환한다.
    std::optional<TransmitBeamCommand> next(std::chrono::milliseconds currentTime);

private:
    bool isReady() const;
    void reset();

    BeamConfig _config;

    OperationState _operationState = OperationState::OFF;
    std::optional<SearchBeamTable> _searchBeamTable;
    RequestQueue _requestQueue;
    std::uint32_t _commandCount = 0;
};

} // namespace beam
