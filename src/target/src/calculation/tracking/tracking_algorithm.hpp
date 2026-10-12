#pragma once

#include "domain/tracking_state.hpp"

#include <target/dto/detection_event.hpp>

#include <chrono>
#include <optional>

namespace target
{

// Strategies retain no per-target state; each Target owns the returned state.
// Calculations leave their inputs unchanged and return nullopt when they fail.
// Successful states use the new observation time; predictions use the requested time.
class ITrackingAlgorithm
{
public:
    virtual ~ITrackingAlgorithm() = default;

    virtual std::optional<TargetTrackingState> initialize(
        const DetectionEvent::Plot& first,
        std::chrono::milliseconds firstTime,
        const DetectionEvent::Plot& second,
        std::chrono::milliseconds secondTime) const = 0;

    virtual std::optional<TargetTrackingState> update(
        const TargetTrackingState& state,
        const DetectionEvent::Plot& plot,
        std::chrono::milliseconds time) const = 0;

    // Prediction does not advance the state or its observation timestamp.
    virtual std::optional<TargetPrediction> predict(
        const TargetTrackingState& state,
        std::chrono::milliseconds time) const = 0;
};

} // namespace target
