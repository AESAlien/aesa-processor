#pragma once

#include "domain/tracking_algorithm.hpp"

namespace target
{

class ConstantVelocityAlgorithm final : public ITrackingAlgorithm
{
public:
    std::optional<TargetTrackingState> initialize(
        const Observation& first,
        const Observation& second
    ) const override;

    std::optional<TargetTrackingState> update(
        const Track& current,
        const Observation& observation
    ) const override;

    std::optional<TargetPrediction> predict(
        const Track& current,
        std::chrono::milliseconds time
    ) const override;
};

} // namespace target
