#pragma once

#include "domain/track.hpp"

#include <cstddef>
#include <limits>
#include <list>
#include <vector>

namespace target
{

// 잠정 트랙이 확정되어도 ID와 등록 순서를 유지한다.
class TrackRepository
{
public:
    // 새 ID는 add()가 성공할 때만 소진된다.
    std::uint32_t nextId() const;
    std::uint32_t add(TentativeTrack tentativeTrack);
    std::uint32_t add(ConfirmedTrack confirmedTrack);
    TentativeTrack* findTentativeTrack(std::uint32_t id);
    const TentativeTrack* findTentativeTrack(std::uint32_t id) const;
    ConfirmedTrack* findConfirmedTrack(std::uint32_t id);
    const ConfirmedTrack* findConfirmedTrack(std::uint32_t id) const;
    bool replace(ConfirmedTrack confirmedTrack);
    bool remove(std::uint32_t id);
    const std::list<Track>& tracks() const;

    // 두 조회 모두 등록 순서로 반환한다. 트랙을 교체하거나 삭제하면 해당 포인터는 무효가 된다.
    std::vector<const ConfirmedTrack*> confirmedTracks(
        std::size_t limit = std::numeric_limits<std::size_t>::max()
    ) const;
    std::vector<const TentativeTrack*> tentativeTracks(
        std::size_t limit = std::numeric_limits<std::size_t>::max()
    ) const;

private:
    std::uint32_t add(Track track);

    std::list<Track> _tracks;
    std::uint32_t _nextId = 1;
};

} // namespace target
