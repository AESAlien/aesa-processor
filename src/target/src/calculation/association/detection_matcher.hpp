#pragma once

#include <optional>

namespace target
{

class Target;

// Shared association checks and NDS distance scoring.
class DetectionMatcher
{
public:
    // Provisional search duplicate gate: chi-square with 3 dimensions, 99%.
    // Tune this default or inject another threshold after checking simulator data.
    static constexpr double DEFAULT_POSITION_NDS_THRESHOLD = 11.345;

    explicit DetectionMatcher(double positionNdsThreshold = DEFAULT_POSITION_NDS_THRESHOLD);

    bool isValidPosition(const Target& observation) const;
    // Return an NDS score inside the search duplicate gate; nullopt means unassociated.
    // Tentative scores use the residual beyond 1000m/s * absolute elapsed time.
    // Tracking scores use the predicted position at or after the latest observation.
    // Older searches use the latest stored position with the same motion allowance.
    std::optional<double> matchesPosition(const Target& observation, const Target& other) const;
    // Return an NDS rank after the speed/Doppler checks; nullopt means ineligible.
    // Confirmation ranking has no additional NDS threshold.
    std::optional<double> matchesConfirmation(const Target& observation, const Target& tentative) const;
    bool matchesTracking(const Target& observation, const Target& tracking) const;

private:
    double _positionNdsThreshold;
};

} // namespace target
