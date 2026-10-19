#pragma once

#include <math/quantity/length.hpp>
#include <math/quantity/linear_velocity.hpp>

#include <chrono>
#include <cstdint>

namespace target
{

struct TargetConfig
{
    TargetConfig();
    TargetConfig(
        math::LinearVelocity maxAssociationSpeed,
        math::LinearVelocity maxInitialDopplerDifference,
        math::Length measurementStandardDeviation,
        double positionNdsThreshold,
        std::chrono::milliseconds confirmationDelay,
        std::chrono::milliseconds trackingInterval,
        std::uint32_t maxConsecutiveTrackingMisses);

    const math::LinearVelocity maxAssociationSpeed;
    const math::LinearVelocity maxInitialDopplerDifference;
    const math::Length measurementStandardDeviation;
    // Provisional chi-square gate for three dimensions at 99%; tune at construction.
    const double positionNdsThreshold;
    const std::chrono::milliseconds confirmationDelay;
    const std::chrono::milliseconds trackingInterval;
    const std::uint32_t maxConsecutiveTrackingMisses;

private:
    // Throws std::invalid_argument when a setting cannot be used by Target.
    void validate() const;
};

} // namespace target
