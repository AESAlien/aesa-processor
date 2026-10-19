#pragma once

#include <beam/dto/beam_request.hpp>
#include <target/dto/detection_event.hpp>
#include <target/dto/track_event.hpp>

#include <memory>
#include <optional>
#include <vector>

namespace target
{

struct TargetConfig;
class TargetRepository;
class ITrackingAlgorithm;
class DetectionMatcher;
class TentativeSelector;
class TrackSelector;

class TargetController
{
public:
    TargetController();

    std::optional<beam::BeamRequest> handleDetection(DetectionEvent detection);
    std::vector<TrackEvent> nextTrackEvents() const;

private:
    std::shared_ptr<const TargetConfig> _config;
    std::shared_ptr<const ITrackingAlgorithm> _trackingAlgorithm;
    std::shared_ptr<TargetRepository> _targetRepository;
    std::shared_ptr<const DetectionMatcher> _matcher;
    // The selectors share the repository and matcher owned by this controller.
    std::shared_ptr<const TentativeSelector> _tentativeSelector;
    std::shared_ptr<const TrackSelector> _trackSelector;
};

} // namespace target
