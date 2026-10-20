#include "scheduler/request_queue.hpp"

namespace beam
{

void RequestQueue::add(const BeamRequest& request)
{
    _requests.emplace(request.transmitTime(), request);
}

std::optional<BeamRequest> RequestQueue::popNextDue(std::chrono::milliseconds currentTime)
{
    // 시각이 currentTime 이하인 요청만 처리 대상이다. 미래 요청은 건드리지 않는다.
    const auto dueEnd = _requests.upper_bound(currentTime);

    if (_requests.begin() == dueEnd)
    {
        return std::nullopt;
    }

    // 가장 이른 시각의 요청들 중 우선순위가 가장 높은 요청을 고른다. 우선순위가 같으면 먼저 들어온 요청을 고른다.
    const auto [earliestBegin, earliestEnd] = _requests.equal_range(_requests.begin()->first);
    auto selected = earliestBegin;
    for (auto candidate = earliestBegin; candidate != earliestEnd; ++candidate)
    {
        if (candidate->second.priority() > selected->second.priority())
        {
            selected = candidate;
        }
    }

    const BeamRequest request = selected->second;
    _requests.erase(selected);
    return request;
}

void RequestQueue::clear()
{
    _requests.clear();
}

} // namespace beam
