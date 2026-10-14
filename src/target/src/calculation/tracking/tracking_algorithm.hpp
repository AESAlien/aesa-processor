#pragma once

#include "domain/tracking_state.hpp"

#include <chrono>
#include <optional>

namespace target
{

class Target;

// Strategies retain no per-target state; each Target owns the returned state.
// Calculations leave their inputs unchanged and return nullopt when they fail.
// Successful states use the received Target's observation time.
class ITrackingAlgorithm
{
public:
    virtual ~ITrackingAlgorithm() = default;

    virtual std::optional<TargetTrackingState> initialize(
        const Target& first,
        const Target& second) const = 0;

    virtual std::optional<TargetTrackingState> update(
        const Target& current,
        const Target& observation) const = 0;

    // Prediction does not advance the state or its observation timestamp.
    virtual std::optional<TargetPrediction> predict(
        const Target& current,
        std::chrono::milliseconds time) const = 0;
};

} // namespace target
