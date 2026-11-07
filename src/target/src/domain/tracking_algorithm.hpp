#pragma once

#include "domain/track_state.hpp"

#include <chrono>
#include <optional>

namespace target
{

class Observation;
class ConfirmedTrack;

// Strategies retain no per-track state and leave their inputs unchanged.
// Calculations return nullopt when no result is available.
// Result constructors validate values; domain objects enforce result timestamps.
class ITrackingAlgorithm
{
public:
    virtual ~ITrackingAlgorithm() = default;

    virtual std::optional<TrackState> initialize(
        const Observation& first,
        const Observation& second
    ) const = 0;

    virtual std::optional<TrackState> update(
        const ConfirmedTrack& current,
        const Observation& observation
    ) const = 0;

    // Prediction does not advance the state or its observation timestamp.
    // Covariance-aware strategies return the predicted 3x3 position covariance
    // in the same Cartesian frame used by the state and position measurements.
    virtual std::optional<TargetPrediction> predict(
        const ConfirmedTrack& current,
        std::chrono::milliseconds time
    ) const = 0;
};

} // namespace target
