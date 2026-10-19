#pragma once

#include <target/config/target_config.hpp>

#include <math/linear_algebra/square_matrix.hpp>

#include <optional>

namespace target
{

class Target;

// Shared association checks and NDS distance scoring.
class DetectionMatcher
{
public:
    explicit DetectionMatcher(TargetConfig config = TargetConfig{});
    // Return an NDS score inside the search duplicate gate; nullopt means unassociated.
    // Tentative scores use the residual beyond maxAssociationSpeed * elapsed time.
    // Tracking scores use the predicted position at or after the latest observation.
    // Older searches use the latest stored position with the same motion allowance.
    std::optional<double> matchesPosition(const Target& observation, const Target& other) const;
    // Return an NDS rank after the speed/Doppler checks; nullopt means ineligible.
    // Confirmation ranking has no additional NDS threshold.
    std::optional<double> matchesConfirmation(const Target& observation, const Target& tentative) const;
    bool matchesTracking(const Target& observation, const Target& tracking) const;

private:
    const TargetConfig _config;
    const math::SquareMatrix _measurementDifferenceCovariance;
};

} // namespace target
