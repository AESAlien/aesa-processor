#pragma once

#include "tracking_state.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>

namespace target
{

class ITrackingAlgorithm;
class TargetRepository;

enum class TargetState
{
    UNREGISTERED,
    TENTATIVE,
    TRACKING
};

class Target
{
public:
    Target(
        math::Length slantRange,
        math::Angle azimuth,
        math::Angle elevation,
        math::LinearVelocity dopplerVelocity,
        float power_db,
        std::chrono::milliseconds time,
        std::shared_ptr<const ITrackingAlgorithm> trackingAlgorithm);

    std::uint32_t id() const;
    TargetState state() const;
    math::Length slantRange() const;
    math::Angle azimuth() const;
    math::Angle elevation() const;
    math::LinearVelocity dopplerVelocity() const;
    float power_db() const;
    std::chrono::milliseconds initialTime() const;
    std::chrono::milliseconds lastObservationTime() const;
    std::chrono::milliseconds lastResultTime() const;
    std::uint32_t consecutiveMissCount() const;
    const std::optional<TargetTrackingState>& trackingState() const;

    bool confirm(const Target& observation);
    bool update(const Target& observation);
    bool recordTrackingMiss(std::chrono::milliseconds time);
    std::optional<TargetPrediction> predict(std::chrono::milliseconds time) const;

private:
    friend class TargetRepository;

    void registerAs(std::uint32_t id);
    bool applyTrackingState(
        const std::optional<TargetTrackingState>& state,
        std::chrono::milliseconds time);
    bool markResultProcessed(std::chrono::milliseconds time);

    std::uint32_t _id = 0;
    // A received Target has no repository identity until a new candidate is selected.
    // Preserve the first observation independently from subsequent tracking updates.
    math::Length _initialSlantRange;
    math::Angle _initialAzimuth;
    math::Angle _initialElevation;
    math::LinearVelocity _initialDopplerVelocity;
    float _initialPower_db;
    std::chrono::milliseconds _initialTime;
    std::chrono::milliseconds _lastResultTime;
    std::uint32_t _consecutiveMissCount = 0;
    std::optional<TargetTrackingState> _trackingState;
    std::shared_ptr<const ITrackingAlgorithm> _trackingAlgorithm;
};

} // namespace target
