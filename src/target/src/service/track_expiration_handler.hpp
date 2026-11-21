#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <vector>

namespace target
{

class TrackRepository;

// 요청한 빔의 결과가 응답 대기 한도를 넘도록 오지 않은 트랙을 삭제한다.
// 트랙에 만료 여부를 질의하고, 만료된 트랙을 저장소에서 제거한다.
class TrackExpirationHandler
{
public:
    explicit TrackExpirationHandler(std::shared_ptr<TrackRepository> repository);

    // currentTime 기준으로 응답 대기 한도를 넘긴 트랙을 삭제하고, 삭제한 트랙 ID를 반환한다.
    std::vector<std::uint32_t> removeExpiredTracks(std::chrono::milliseconds currentTime);

private:
    std::shared_ptr<TrackRepository> _trackRepository;
};

} // namespace target
