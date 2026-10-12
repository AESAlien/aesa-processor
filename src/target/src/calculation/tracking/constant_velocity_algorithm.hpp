#pragma once

#include "tracking_algorithm.hpp"

namespace target
{

class ConstantVelocityAlgorithm final : public ITrackingAlgorithm
{
public:
    std::optional<TargetTrackingState> initialize(
        const DetectionEvent::Plot& first,
        std::chrono::milliseconds firstTime,
        const DetectionEvent::Plot& second,
        std::chrono::milliseconds secondTime) const override;

    std::optional<TargetTrackingState> update(
        const TargetTrackingState& state,
        const DetectionEvent::Plot& plot,
        std::chrono::milliseconds time) const override;

    std::optional<TargetPrediction> predict(
        const TargetTrackingState& state,
        std::chrono::milliseconds time) const override;
};

} // namespace target
