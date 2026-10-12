#pragma once

#include <target/dto/detection_event.hpp>

namespace target
{

struct TargetPrediction;

// Shared association boundary. NDS and its input covariance are not implemented yet.
class DetectionMatcher
{
public:
    bool isValidPosition(const DetectionEvent::Plot& plot) const;
    bool matchesPosition(const DetectionEvent::Plot& plot, const DetectionEvent::Plot& other) const;
    bool matchesPosition(const DetectionEvent::Plot& plot, const TargetPrediction& prediction) const;
};

} // namespace target
