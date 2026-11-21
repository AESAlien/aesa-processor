#pragma once

#include "observation.hpp"
#include "track_prediction.hpp"
#include "track_state.hpp"

#include <target/config/track_lifecycle_config.hpp>

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
        TrackLifecycleConfig lifecycleConfig,
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
    std::uint32_t consecutiveMissCount() const;
    const std::shared_ptr<const TrackState>& state() const noexcept;

    // 미탐을 포함한 마지막 처리 시각보다 새로운 결과만 처리할 수 있다.
    bool canProcessResult(std::chrono::milliseconds resultTime) const;
    // 미탐을 포함한 마지막 결과를 기준으로 판단한다. 응답 대기 한도 시각까지는 유효하다.
    bool isExpired(std::chrono::milliseconds currentTime) const;
    // 미탐을 기록하기 전에 이번 미탐으로 한도에 도달할지 판단한다.
    bool willExpireOnNextMiss() const;
    // 이미 기록한 연속 미탐 횟수가 한도에 도달했는지 판단한다.
    bool hasReachedMissLimit() const;
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
    const TrackLifecycleConfig _lifecycleConfig;
    std::chrono::milliseconds _lastResultTime;
    std::uint32_t _consecutiveMissCount = 0;
    std::shared_ptr<const TrackState> _state;
    std::any _algorithmState;
    std::shared_ptr<const ITrackingAlgorithm> _trackingAlgorithm;
};

} // namespace target
