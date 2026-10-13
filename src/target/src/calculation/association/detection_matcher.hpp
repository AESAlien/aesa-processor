#pragma once

namespace target
{

class Target;

// Shared association boundary. NDS and its input covariance are not implemented yet.
class DetectionMatcher
{
public:
    bool isValidPosition(const Target& observation) const;
    bool matchesPosition(const Target& observation, const Target& other) const;
    bool matchesConfirmation(const Target& observation, const Target& tentative) const;
    bool matchesTracking(const Target& observation, const Target& tracking) const;
};

} // namespace target
