#include "confirmed_track.hpp"

#include "domain/tracking/tracking_algorithm.hpp"

#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace target
{

ConfirmedTrack::ConfirmedTrack(
    std::uint32_t id,
    std::chrono::milliseconds initialTime,
    TrackState state,
    TrackLifecycleConfig lifecycleConfig,
    std::shared_ptr<const ITrackingAlgorithm> trackingAlgorithm,
    std::any algorithmState
) : _id(id),
    _lifecycleConfig(std::move(lifecycleConfig)),
    _lastResultTime(state.timestamp()),
    _state(std::move(state)),
    _algorithmState(std::move(algorithmState)),
    _trackingAlgorithm(std::move(trackingAlgorithm))
{
    if (_id == 0 || !_trackingAlgorithm)
    {
        throw std::invalid_argument("Confirmed target requires a nonzero ID and tracking algorithm");
    }
    if (initialTime.count() < 0 || _lastResultTime <= initialTime)
    {
        throw std::invalid_argument("Confirmed target requires a state after its nonnegative initial time");
    }
}

std::uint32_t ConfirmedTrack::id() const
{
    return _id;
}

const TrackState& ConfirmedTrack::state() const noexcept
{
    return _state;
}

const std::any& ITrackingAlgorithm::algorithmState(const ConfirmedTrack& current) noexcept
{
    return current._algorithmState;
}

bool ConfirmedTrack::canProcessResult(std::chrono::milliseconds resultTime) const
{
    return resultTime > _lastResultTime;
}

bool ConfirmedTrack::isExpired(std::chrono::milliseconds currentTime) const
{
    return currentTime > _lastResultTime + _lifecycleConfig.trackingInterval() + _lifecycleConfig.responseTimeout();
}

bool ConfirmedTrack::willExpireOnNextMiss() const
{
    return _consecutiveMissCount >= _lifecycleConfig.maxConsecutiveTrackingMisses() - 1;
}

bool ConfirmedTrack::hasReachedMissLimit() const
{
    return _consecutiveMissCount >= _lifecycleConfig.maxConsecutiveTrackingMisses();
}

bool ConfirmedTrack::update(const Observation& observation)
{
    const auto time = observation.observationTime();
    if (!canProcessResult(time))
    {
        return false;
    }

    auto result = _trackingAlgorithm->update(*this, observation);
    if (!result)
    {
        return false;
    }

    auto& [state, nextAlgorithmState] = *result;
    if (state.timestamp() != time)
    {
        throw std::invalid_argument("Tracking state must use the observation time");
    }

    // 계산과 검증이 끝난 결과를 예외 없이 반영해 기존 상태와 처리 이력의 일관성을 보장한다.
    static_assert(std::is_nothrow_move_assignable_v<TrackState>);
    static_assert(std::is_nothrow_move_assignable_v<std::any>);
    _state = std::move(state);
    _algorithmState = std::move(nextAlgorithmState);
    _lastResultTime = time;
    _consecutiveMissCount = 0;
    return true;
}

bool ConfirmedTrack::recordTrackingMiss(std::chrono::milliseconds time)
{
    if (!canProcessResult(time))
    {
        return false;
    }

    _lastResultTime = time;
    if (_consecutiveMissCount < std::numeric_limits<std::uint32_t>::max())
    {
        ++_consecutiveMissCount;
    }

    return true;
}

std::optional<TrackPrediction> ConfirmedTrack::predict(std::chrono::milliseconds time) const
{
    if (time < _state.timestamp())
    {
        return std::nullopt;
    }

    return validatePrediction(_trackingAlgorithm->predict(*this, time), time);
}

std::optional<TrackPrediction> ConfirmedTrack::validatePrediction(
    std::optional<TrackPrediction> prediction,
    std::chrono::milliseconds time
) {
    if (prediction && prediction->timestamp() != time)
    {
        throw std::invalid_argument("Prediction must use the requested time");
    }
    
    return prediction;
}

} // namespace target
