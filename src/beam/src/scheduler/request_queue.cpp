#include "scheduler/request_queue.hpp"

#include <iterator>

namespace beam
{
namespace
{
using namespace std::chrono_literals;

constexpr std::chrono::milliseconds CONFIRMATION_MAX_DELAY = 20ms;
constexpr std::chrono::milliseconds TRACKING_MAX_DELAY = 30ms;

std::chrono::milliseconds maxDelay(BeamRequest::BeamType beamType)
{
    switch (beamType)
    {
    case BeamRequest::BeamType::CONFIRMATION:
        return CONFIRMATION_MAX_DELAY;

    case BeamRequest::BeamType::TRACKING:
        return TRACKING_MAX_DELAY;
    }

    // 알 수 없는 종류는 허용 지연을 0으로 보아 곧바로 만료시킨다.
    return 0ms;
}

bool isExpired(const BeamRequest& request, std::chrono::milliseconds currentTime)
{
    return currentTime - request.timestamp >= maxDelay(request.beamType);
}
} // namespace

void RequestQueue::add(const BeamRequest& request)
{
    _requests.emplace(request.timestamp, request);
}

std::optional<BeamRequest> RequestQueue::popNextDue(std::chrono::milliseconds currentTime)
{
    // 시각이 currentTime 이하인 요청만 처리 대상이다. 미래 요청은 건드리지 않는다.
    const auto dueEnd = _requests.upper_bound(currentTime);

    for (auto it = _requests.begin(); it != dueEnd;)
    {
        it = isExpired(it->second, currentTime) ? _requests.erase(it) : std::next(it);
    }

    if (_requests.begin() == dueEnd)
    {
        return std::nullopt;
    }

    // 가장 이른 시각의 요청들 중 확인 빔이 있으면 그것을, 없으면 먼저 들어온 요청을 고른다.
    const auto [first, last] = _requests.equal_range(_requests.begin()->first);
    auto selected = first;
    for (auto it = first; it != last; ++it)
    {
        if (it->second.beamType == BeamRequest::BeamType::CONFIRMATION)
        {
            selected = it;
            break;
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
