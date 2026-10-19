#include "scheduler/request_queue.hpp"

#include "scheduler/request_beam_policy.hpp"

namespace beam
{
namespace
{
int priorityOf(const BeamRequest& request)
{
    return requestBeamPolicyOf(request.beamType()).priority();
}
} // namespace

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
    const auto [first, last] = _requests.equal_range(_requests.begin()->first);
    auto selected = first;
    for (auto it = first; it != last; ++it)
    {
        if (priorityOf(it->second) > priorityOf(selected->second))
        {
            selected = it;
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

std::size_t RequestQueue::size() const
{
    return _requests.size();
}

bool RequestQueue::empty() const
{
    return _requests.empty();
}

} // namespace beam
