#pragma once

#include "domain/track_prediction.hpp"
#include "domain/tracking/tracking_result.hpp"

#include <any>
#include <chrono>
#include <optional>

namespace target
{

class Observation;
class ConfirmedTrack;

// 알고리즘은 트랙별 상태를 보관하지 않으며, 결과를 계산할 수 없으면 nullopt를 반환한다.
class ITrackingAlgorithm
{
public:
    virtual ~ITrackingAlgorithm() = default;

    // 전략별 데이터는 성공한 초기화나 갱신과 함께 트랙에 저장된다.
    virtual std::optional<TrackingResult> initialize(
        const Observation& first,
        const Observation& second
    ) const = 0;

    virtual std::optional<TrackingResult> update(
        const ConfirmedTrack& current,
        const Observation& observation
    ) const = 0;

    virtual std::optional<TrackPrediction> predict(
        const ConfirmedTrack& current,
        std::chrono::milliseconds time
    ) const = 0;

protected:
    static const std::any& algorithmState(const ConfirmedTrack& current) noexcept;
};

} // namespace target
