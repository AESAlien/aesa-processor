#pragma once

#include "observation.hpp"
#include "track_state.hpp"

#include <cstdint>
#include <memory>
#include <optional>

namespace target
{

class ITrackingAlgorithm;

// A confirmed target always owns a non-null, immutable tracking state.
class ConfirmedTrack
{
public:
    ConfirmedTrack(
        std::uint32_t id,
        std::chrono::milliseconds initialTime,
        TrackState state,
        std::shared_ptr<const ITrackingAlgorithm> trackingAlgorithm
    );

    std::uint32_t id() const;
    math::Distance slantRange() const;
    math::CircularAngle azimuth() const;
    math::TiltAngle elevation() const;
    const math::SphericalPosition& sphericalPosition() const;
    math::LinearVelocity dopplerVelocity() const;
    float power_db() const;
    std::chrono::milliseconds initialTime() const;
    std::chrono::milliseconds lastObservationTime() const;
    std::chrono::milliseconds lastResultTime() const;
    std::uint32_t consecutiveMissCount() const;
    const std::shared_ptr<const TrackState>& trackingState() const;

    bool update(const Observation& observation);
    bool recordTrackingMiss(std::chrono::milliseconds time);
    std::optional<TargetPrediction> predict(std::chrono::milliseconds time) const;

private:
    bool applyTrackingState(
        std::optional<TrackState> state,
        std::chrono::milliseconds time
    );

    const std::uint32_t _id;
    const std::chrono::milliseconds _initialTime;
    std::chrono::milliseconds _lastResultTime;
    std::uint32_t _consecutiveMissCount = 0;
    std::shared_ptr<const TrackState> _trackingState;
    std::shared_ptr<const ITrackingAlgorithm> _trackingAlgorithm;
};

} // namespace target
