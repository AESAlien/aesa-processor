#pragma once

#include "scheduler/beam_request.hpp"

#include <chrono>
#include <cstddef>
#include <map>
#include <optional>

namespace beam
{

// 확인/추적 빔 요청을 시각 순으로 보관한다.
class RequestQueue
{
public:
    void add(const BeamRequest& request);

    // 시각이 currentTime 이하인 요청 중 가장 이른 요청을 꺼낸다.
    // 시각이 같으면 확인 빔을 먼저, 종류도 같으면 먼저 들어온 요청을 꺼낸다.
    // 꺼낼 요청이 없으면 std::nullopt를 반환한다.
    std::optional<BeamRequest> popNextDue(std::chrono::milliseconds currentTime);

    void clear();
    std::size_t size() const;
    bool empty() const;

private:
    // 같은 키의 요소는 삽입 순서가 유지된다.
    std::multimap<std::chrono::milliseconds, BeamRequest> _requests;
};

} // namespace beam
