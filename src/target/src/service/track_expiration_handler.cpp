#include "service/track_expiration_handler.hpp"

#include "repository/track_repository.hpp"

#include <stdexcept>
#include <utility>
#include <variant>

namespace target
{

TrackExpirationHandler::TrackExpirationHandler(
    std::shared_ptr<TrackRepository> repository
) : _trackRepository(std::move(repository))
{
    if (!_trackRepository)
    {
        throw std::invalid_argument("Track expiration handler requires a repository");
    }
}

std::vector<std::uint32_t> TrackExpirationHandler::removeExpiredTracks(std::chrono::milliseconds currentTime)
{
    std::vector<std::uint32_t> expiredIds;
    for (const auto& track : _trackRepository->tracks())
    {
        std::visit(
            [&](const auto& value)
            {
                if (value.isExpired(currentTime))
                {
                    expiredIds.push_back(value.id());
                }
            },
            track
        );
    }

    // 순회 중에 삭제하면 반복자가 무효가 되므로 ID를 모은 뒤 삭제한다.
    for (const auto id : expiredIds)
    {
        _trackRepository->removeById(id);
    }
    return expiredIds;
}

} // namespace target
