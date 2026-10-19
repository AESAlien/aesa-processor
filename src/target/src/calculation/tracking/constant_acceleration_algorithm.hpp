#pragma once

#include "tracking_algorithm.hpp"

#include <target/config/target_config.hpp>

namespace target
{

// Nine-state linear Cartesian Kalman filter with constant acceleration and Q = 0.
class ConstantAccelerationAlgorithm final : public ITrackingAlgorithm
{
public:
    explicit ConstantAccelerationAlgorithm(TargetConfig config);

    std::optional<TargetTrackingState> initialize(
        const Target& first,
        const Target& second) const override;

    std::optional<TargetTrackingState> update(
        const Target& current,
        const Target& observation) const override;

    std::optional<TargetPrediction> predict(
        const Target& current,
        std::chrono::milliseconds time) const override;

private:
    const TargetConfig _config;
};

} // namespace target
