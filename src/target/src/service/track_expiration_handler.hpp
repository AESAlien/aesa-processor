#pragma once

#include "domain/track.hpp"

#include <target/config/track_management_config.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <vector>

namespace target
{

class TrackRepository;

// 요청한 빔의 결과가 응답 대기 한도를 넘도록 오지 않은 트랙을 삭제한다.
// 결과가 오지 않으면 다음 빔도 요청되지 않아 트랙이 스스로 갱신되거나 만료될 수 없다.
class TrackExpirationHandler
{
public:
    TrackExpirationHandler(TrackManagementConfig config, std::shared_ptr<TrackRepository> repository);

    // currentTime 기준으로 응답 대기 한도를 넘긴 트랙을 삭제하고, 삭제한 트랙 ID를 반환한다.
    std::vector<std::uint32_t> removeExpiredTracks(std::chrono::milliseconds currentTime);

private:
    std::chrono::milliseconds expectedResultTime(const Track& track) const;

    const TrackManagementConfig _config;
    std::shared_ptr<TrackRepository> _trackRepository;
};

} // namespace target
