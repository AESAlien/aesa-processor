#pragma once

#include "tracking_algorithm.hpp"

namespace target
{

class ConstantVelocityAlgorithm final : public ITrackingAlgorithm
{
public:
    std::optional<TargetTrackingState> initialize(
        const Target& first,
        const Target& second) const override;

    std::optional<TargetTrackingState> update(
        const Target& current,
        const Target& observation) const override;

    std::optional<TargetPrediction> predict(
        const Target& current,
        std::chrono::milliseconds time) const override;
};

} // namespace target
