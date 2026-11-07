#pragma once

#include "domain/tracking_algorithm.hpp"

namespace target
{

class ConstantVelocityAlgorithm final : public ITrackingAlgorithm
{
public:
    std::optional<TrackState> initialize(
        const Observation& first,
        const Observation& second
    ) const override;

    std::optional<TrackState> update(
        const ConfirmedTrack& current,
        const Observation& observation
    ) const override;

    std::optional<TargetPrediction> predict(
        const ConfirmedTrack& current,
        std::chrono::milliseconds time
    ) const override;
};

} // namespace target
