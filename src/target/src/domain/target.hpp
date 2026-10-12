#pragma once

#include <target/dto/detection_event.hpp>

#include "tracking_state.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>

namespace target
{

class ITrackingAlgorithm;

enum class TargetState
{
    TENTATIVE,
    TRACKING
};

class Target
{
public:
    Target(
        std::uint32_t id,
        const DetectionEvent::Plot& initialPlot,
        std::chrono::milliseconds initialTime,
        std::shared_ptr<const ITrackingAlgorithm> trackingAlgorithm);

    std::uint32_t id() const;
    TargetState state() const;
    DetectionEvent::Plot initialPlot() const;
    DetectionEvent::Plot lastPlot() const;
    std::chrono::milliseconds initialTime() const;
    std::chrono::milliseconds lastObservationTime() const;
    std::chrono::milliseconds lastResultTime() const;
    std::uint32_t consecutiveMissCount() const;
    const std::optional<TargetTrackingState>& trackingState() const;

    bool canConfirm(const DetectionEvent::Plot& plot, std::chrono::milliseconds time) const;
    bool confirm(const DetectionEvent::Plot& plot, std::chrono::milliseconds time);
    bool update(const DetectionEvent::Plot& plot, std::chrono::milliseconds time);
    bool recordTrackingMiss(std::chrono::milliseconds time);
    bool markResultProcessed(std::chrono::milliseconds time);
    std::optional<TargetPrediction> predict(std::chrono::milliseconds time) const;

private:
    std::optional<TargetTrackingState> initializeTrackingState(
        const DetectionEvent::Plot& plot,
        std::chrono::milliseconds time) const;

    std::uint32_t _id;
    // Preserve the initial detection independently from subsequent tracking updates.
    // Copying values also keeps Target assignable despite the DTO's const fields.
    math::Distance _initialSlantRange;
    math::Angle _initialAzimuth;
    math::Angle _initialElevation;
    math::Velocity _initialDopplerVelocity;
    float _initialPowerDb;
    std::chrono::milliseconds _initialTime;
    std::chrono::milliseconds _lastResultTime;
    std::uint32_t _consecutiveMissCount = 0;
    std::optional<TargetTrackingState> _trackingState;
    std::shared_ptr<const ITrackingAlgorithm> _trackingAlgorithm;
};

} // namespace target
