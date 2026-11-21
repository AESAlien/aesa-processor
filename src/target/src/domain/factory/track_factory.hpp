#pragma once

#include "domain/tentative_track.hpp"

#include <cstdint>
#include <memory>

namespace target
{

class TrackFactory
{
public:
    TrackFactory(std::shared_ptr<const ITrackingAlgorithm> trackingAlgorithm, TrackLifecycleConfig lifecycleConfig);

    TrackFactory(const TrackFactory&) = delete;
    TrackFactory& operator=(const TrackFactory&) = delete;
    TrackFactory(TrackFactory&&) noexcept = default;
    TrackFactory& operator=(TrackFactory&&) noexcept = default;

    // 생성에 성공하면 새 ID를 소진한다. 저장 여부와 관계없이 발급한 ID는 재사용하지 않는다.
    TentativeTrack createTentative(Observation observation);

private:
    std::shared_ptr<const ITrackingAlgorithm> _trackingAlgorithm;
    TrackLifecycleConfig _lifecycleConfig;
    std::uint32_t _nextId = 1;
};

} // namespace target
