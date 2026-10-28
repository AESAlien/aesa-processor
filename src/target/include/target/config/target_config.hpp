#pragma once

#include <math/quantity/length.hpp>
#include <math/quantity/linear_velocity.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>

namespace target
{

class TargetConfig
{
public:
    // Maximum console message capacity; a smaller reporting limit can be injected.
    static constexpr std::size_t MAXIMUM_REPORTED_TRACK_COUNT = 100;

    TargetConfig(
        math::LinearVelocity maxAssociationSpeed,
        math::LinearVelocity maxInitialDopplerDifference,
        math::Length measurementStandardDeviation,
        double positionNdsThreshold,
        std::chrono::milliseconds confirmationDelay,
        std::chrono::milliseconds trackingInterval,
        std::uint32_t maxConsecutiveTrackingMisses,
        double initialAccelerationStandardDeviation,
        std::size_t maxReportedTrackCount);

    const math::LinearVelocity& maxAssociationSpeed() const noexcept;
    const math::LinearVelocity& maxInitialDopplerDifference() const noexcept;
    const math::Length& measurementStandardDeviation() const noexcept;
    // Provisional chi-square gate for three dimensions at 99%; tune at construction.
    double positionNdsThreshold() const noexcept;
    std::chrono::milliseconds confirmationDelay() const noexcept;
    std::chrono::milliseconds trackingInterval() const noexcept;
    std::uint32_t maxConsecutiveTrackingMisses() const noexcept;
    // Per-axis initial acceleration uncertainty in m/s^2; a tunable starting value.
    double initialAccelerationStandardDeviation() const noexcept;
    std::size_t maxReportedTrackCount() const noexcept;

private:
    // Throws std::invalid_argument when a setting cannot be used by Target.
    math::LinearVelocity _maxAssociationSpeed;
    math::LinearVelocity _maxInitialDopplerDifference;
    math::Length _measurementStandardDeviation;
    double _positionNdsThreshold;
    std::chrono::milliseconds _confirmationDelay;
    std::chrono::milliseconds _trackingInterval;
    std::uint32_t _maxConsecutiveTrackingMisses;
    double _initialAccelerationStandardDeviation;
    std::size_t _maxReportedTrackCount;
    void validate() const;
};

} // namespace target
