#pragma once

#include <math/quantity/length.hpp>
#include <math/quantity/linear_velocity.hpp>

#include <chrono>
#include <cstdint>

namespace target
{

struct TargetConfig
{
    // Internal settings constructed and injected by TargetController.
    TargetConfig();

    // Throws std::invalid_argument when a setting cannot be used by Target.
    void validate() const;

    math::LinearVelocity maxAssociationSpeed;
    math::LinearVelocity maxInitialDopplerDifference;
    math::Length measurementStandardDeviation;
    // Provisional chi-square gate for three dimensions at 99%; tune as needed.
    double positionNdsThreshold;
    std::chrono::milliseconds confirmationDelay;
    std::chrono::milliseconds trackingInterval;
    std::uint32_t maxConsecutiveTrackingMisses;
};

} // namespace target
