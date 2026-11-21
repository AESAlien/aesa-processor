#pragma once

#include "observation.hpp"
#include "confirmed_track.hpp"
#include "domain/tracking/tracking_result.hpp"

namespace target
{

class TentativeTrack
{
public:
    TentativeTrack(
        std::uint32_t id,
        Observation observation,
        TrackLifecycleConfig lifecycleConfig,
        std::shared_ptr<const ITrackingAlgorithm> trackingAlgorithm
    );

    std::uint32_t id() const;
    const Observation& observation() const;

    // 최초 관측 이후의 결과만 처리할 수 있다. 같은 시각과 이전 시각은 거부한다.
    bool canProcessResult(std::chrono::milliseconds resultTime) const;
    // 확인 결과의 응답 대기 한도를 넘었는지 판단한다. 한도 시각까지는 유효하다.
    bool isExpired(std::chrono::milliseconds currentTime) const;
    std::optional<TrackingResult> initialize(const Observation& observation) const;
    std::optional<ConfirmedTrack> confirm(const Observation& observation) const;

private:
    const std::uint32_t _id;
    const Observation _observation;
    const TrackLifecycleConfig _lifecycleConfig;
    const std::shared_ptr<const ITrackingAlgorithm> _trackingAlgorithm;
};

} // namespace target
