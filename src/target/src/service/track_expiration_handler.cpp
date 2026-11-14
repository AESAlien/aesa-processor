#include "service/track_expiration_handler.hpp"

#include "repository/track_repository.hpp"

#include <stdexcept>
#include <utility>
#include <variant>

namespace target
{

TrackExpirationHandler::TrackExpirationHandler(
    TrackManagementConfig config,
    std::shared_ptr<TrackRepository> repository
) : _config(std::move(config)),
    _trackRepository(std::move(repository))
{
    if (!_trackRepository)
    {
        throw std::invalid_argument("Track expiration handler requires a repository");
    }
}

std::vector<std::uint32_t> TrackExpirationHandler::removeExpiredTracks(std::chrono::milliseconds currentTime)
{
    // 연속 미탐으로 만료되는 시간(추적 주기 x 연속 미탐 한도)을 응답 대기 한도로 쓴다.
    // 두 설정 중 하나를 바꾸면 응답 대기 한도도 함께 바뀐다.
    const auto responseTimeout = _config.trackingInterval() * _config.maxConsecutiveTrackingMisses();

    std::vector<std::uint32_t> expiredIds;
    for (const auto& track : _trackRepository->tracks())
    {
        if (currentTime > expectedResultTime(track) + responseTimeout)
        {
            expiredIds.push_back(std::visit(
                [](const auto& value)
                {
                    return value.id();
                },
                track
            ));
        }
    }

    // 순회 중에 삭제하면 반복자가 무효가 되므로 ID를 모은 뒤 삭제한다.
    for (const auto id : expiredIds)
    {
        _trackRepository->remove(id);
    }
    return expiredIds;
}

std::chrono::milliseconds TrackExpirationHandler::expectedResultTime(const Track& track) const
{
    if (const auto* tentativeTrack = std::get_if<TentativeTrack>(&track))
    {
        return tentativeTrack->lastResultTime() + _config.confirmationDelay();
    }
    return std::get<ConfirmedTrack>(track).lastResultTime() + _config.trackingInterval();
}

} // namespace target