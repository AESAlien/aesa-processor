#pragma once

#include "domain/tracking/tracking_algorithm.hpp"

namespace target
{

// 가속도는 0으로 두고 공분산은 추정하지 않는다.
class ConstantVelocityAlgorithm final : public ITrackingAlgorithm
{
public:
    std::optional<TrackingResult> initialize(
        const Observation& first,
        const Observation& second
    ) const override;

    std::optional<TrackingResult> update(
        const ConfirmedTrack& current,
        const Observation& observation
    ) const override;

    std::optional<TrackPrediction> predict(
        const ConfirmedTrack& current,
        std::chrono::milliseconds time
    ) const override;
};

} // namespace target
