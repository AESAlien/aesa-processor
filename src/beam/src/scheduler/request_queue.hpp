#pragma once

#include "scheduler/beam_request.hpp"

#include <chrono>
#include <map>
#include <optional>

namespace aesa_processor::beam
{

// 확인/추적 빔 요청을 시각 순으로 보관한다.
class RequestQueue
{
public:
    void add(const BeamRequest& request);

    // 시각이 currentTime 이하인 요청 중 가장 이른 요청을 꺼낸다.
    // 시각이 같으면 우선순위가 높은 요청(확인 빔이 추적 빔보다 높다)을 먼저, 우선순위도 같으면 먼저 들어온 요청을 꺼낸다.
    // 꺼낼 요청이 없으면 std::nullopt를 반환한다.
    std::optional<BeamRequest> popNextDue(std::chrono::milliseconds currentTime);

    void clear();

private:
    // 같은 키의 요소는 삽입 순서가 유지된다.
    std::multimap<std::chrono::milliseconds, BeamRequest> _requests;
};

} // namespace aesa_processor::beam
