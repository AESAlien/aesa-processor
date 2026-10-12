#include "scheduler/request_queue.hpp"

#include <iterator>

namespace beam
{
namespace
{
using namespace std::chrono_literals;

std::chrono::milliseconds maxDelay(const RequestQueueConfig& config, BeamRequest::BeamType beamType)
{
    switch (beamType)
    {
    case BeamRequest::BeamType::CONFIRMATION:
        return config.confirmationMaxDelay;

    case BeamRequest::BeamType::TRACKING:
        return config.trackingMaxDelay;
    }

    // 알 수 없는 종류는 허용 지연을 0으로 보아 곧바로 만료시킨다.
    return 0ms;
}

bool isExpired(const RequestQueueConfig& config, const BeamRequest& request, std::chrono::milliseconds currentTime)
{
    return currentTime - request.transmitTime >= maxDelay(config, request.beamType);
}
} // namespace

RequestQueue::RequestQueue(const RequestQueueConfig& config)
{
    _config = config;
}

void RequestQueue::add(const BeamRequest& request)
{
    _requests.emplace(request.transmitTime, request);
}

std::optional<BeamRequest> RequestQueue::popNextDue(std::chrono::milliseconds currentTime)
{
    // 시각이 currentTime 이하인 요청만 처리 대상이다. 미래 요청은 건드리지 않는다.
    const auto dueEnd = _requests.upper_bound(currentTime);

    for (auto it = _requests.begin(); it != dueEnd;)
    {
        it = isExpired(_config, it->second, currentTime) ? _requests.erase(it) : std::next(it);
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
