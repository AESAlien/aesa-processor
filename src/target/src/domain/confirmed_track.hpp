#pragma once

#include "observation.hpp"
#include "track_prediction.hpp"
#include "track_state.hpp"

#include <any>
#include <cstdint>
#include <memory>
#include <optional>

namespace target
{

class ITrackingAlgorithm;
class IPositionUncertainty;

// 확정 트랙은 항상 유효한 불변 추적 상태를 보유한다.
class ConfirmedTrack
{
public:
    ConfirmedTrack(
        std::uint32_t id,
        std::chrono::milliseconds initialTime,
        TrackState state,
        std::shared_ptr<const ITrackingAlgorithm> trackingAlgorithm,
        std::any algorithmState = {}
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
    const std::shared_ptr<const TrackState>& state() const noexcept;

    bool update(const Observation& observation);
    bool recordTrackingMiss(std::chrono::milliseconds time);
    std::optional<TrackPrediction> predict(std::chrono::milliseconds time) const;

private:
    friend class ITrackingAlgorithm;
    friend class IPositionUncertainty;

    static std::optional<TrackPrediction> validatePrediction(
        std::optional<TrackPrediction> prediction,
        std::chrono::milliseconds time
    );

    const std::uint32_t _id;
    const std::chrono::milliseconds _initialTime;
    std::chrono::milliseconds _lastResultTime;
    std::uint32_t _consecutiveMissCount = 0;
    std::shared_ptr<const TrackState> _state;
    std::any _algorithmState;
    std::shared_ptr<const ITrackingAlgorithm> _trackingAlgorithm;
};

} // namespace target
