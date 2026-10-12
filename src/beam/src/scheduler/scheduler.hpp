#pragma once

#include "attitude/radar_attitude.hpp"
#include "scheduler/beam_table.hpp"
#include "scheduler/request_queue.hpp"

#include <beam/domain/operation_state.hpp>
#include <beam/dto/beam_command.hpp>
#include <beam/dto/beam_request.hpp>

#include <chrono>
#include <cstddef>
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
//
// 상태:
// - OFF에서는 모든 입력을 무시하고 next()는 std::nullopt를 반환한다.
// - OFF에서 ON으로 바뀌는 순간 모든 상태를 초기화한다.
// - ON이고 자세정보를 받아 BeamTable이 만들어진 상태에서만 빔을 송신한다.
class Scheduler
{
public:
    void setOperationState(OperationState state);

    // ON이고 BeamTable이 아직 없을 때만 받아들이고, 그 외에는 무시한다.
    // 잘못된 자세정보면 AttitudeError를 던지며, 이때 상태는 변하지 않는다.
    void setAttitude(const RadarAttitude& attitude);

    // 준비 상태(ON이고 BeamTable이 있음)가 아니면 무시한다.
    void addRequest(const BeamRequest& request);

    // 준비 상태가 아니면 std::nullopt를 반환한다.
    // 송신할 요청이 있으면 요청 빔을, 없으면 탐색 빔을 반환한다.
    std::optional<BeamCommand> next(std::chrono::milliseconds currentTime);

private:
    bool isReady() const;
    void reset();

    OperationState _operationState = OperationState::OFF;
    std::optional<BeamTable> _beamTable;
    RequestQueue _requestQueue;
    std::size_t _searchCursor = 0;
    std::uint32_t _commandCount = 0;
};

} // namespace beam
