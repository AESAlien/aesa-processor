#pragma once

#include <math/quantity/length.hpp>
#include <math/quantity/linear_acceleration.hpp>
#include <math/quantity/linear_velocity.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>

namespace target
{

struct TargetConfig
{
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
        math::LinearAcceleration initialAccelerationStandardDeviation,
        std::size_t maxReportedTrackCount,
        math::LinearAcceleration processAccelerationStandardDeviation);

    const math::LinearVelocity maxAssociationSpeed;
    const math::LinearVelocity maxInitialDopplerDifference;
    const math::Length measurementStandardDeviation;
    // Provisional chi-square gate for three dimensions at 99%; tune at construction.
    const double positionNdsThreshold;
    const std::chrono::milliseconds confirmationDelay;
    const std::chrono::milliseconds trackingInterval;
    const std::uint32_t maxConsecutiveTrackingMisses;
    // Per-axis initial acceleration uncertainty in m/s^2; a tunable starting value.
    const math::LinearAcceleration initialAccelerationStandardDeviation;
    const std::size_t maxReportedTrackCount;
    // Per-axis acceleration disturbance (1 sigma) for each prediction interval.
    const math::LinearAcceleration processAccelerationStandardDeviation;

private:
    // Throws std::invalid_argument when a setting cannot be used by Target.
    void validate() const;
};

} // namespace target
