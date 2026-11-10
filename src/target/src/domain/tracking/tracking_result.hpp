#pragma once

#include "domain/track_state.hpp"

#include <any>

namespace target
{

// 추정 상태와 전략별 데이터를 한 계산 결과로 전달한다.
struct TrackingResult
{
    TrackState state;
    std::any algorithmState;
};

} // namespace target
