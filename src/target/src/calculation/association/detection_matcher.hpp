#pragma once

#include <target/config/target_config.hpp>

#include <math/linear_algebra/square_matrix.hpp>

#include <optional>

namespace aesa_processor::target
{

class Target;

// Shared association checks and NDS distance scoring.
class DetectionMatcher
{
public:
    explicit DetectionMatcher(TargetConfig config = TargetConfig{});
    // Return an NDS score inside the search duplicate gate; nullopt means unassociated.
    // Tentative scores use the residual beyond maxAssociationSpeed * elapsed time.
    // Tracking scores use predicted position covariance + measurement covariance.
    // Older searches use the stored position covariance with the motion allowance.
    std::optional<double> matchesPosition(const Target& observation, const Target& other) const;
    // Return an NDS rank after the speed/Doppler checks; nullopt means ineligible.
    // Confirmation ranking has no additional NDS threshold.
    std::optional<double> matchesConfirmation(const Target& observation, const Target& tentative) const;
    // Return an NDS score inside the position gate around the prediction at observation time.
    // Algorithms without predicted covariance retain the legacy two-measurement approximation.
    std::optional<double> matchesTracking(const Target& observation, const Target& tracking) const;

private:
    const TargetConfig _config;
    const aesa_processor::math::SquareMatrix _measurementCovariance;
    const aesa_processor::math::SquareMatrix _measurementDifferenceCovariance;
};

} // namespace aesa_processor::target
