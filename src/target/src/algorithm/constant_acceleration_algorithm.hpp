#pragma once

#include "domain/tracking_algorithm.hpp"

#include <target/config/measurement_config.hpp>
#include <target/config/tracking_config.hpp>

namespace target
{

// Nine-state CA Kalman filter with configurable per-interval acceleration noise.
class ConstantAccelerationAlgorithm final : public ITrackingAlgorithm
{
public:
    ConstantAccelerationAlgorithm(TrackingConfig config, MeasurementConfig measurementConfig);

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

private:
    const TrackingConfig _config;
    const MeasurementConfig _measurementConfig;
};

} // namespace target
