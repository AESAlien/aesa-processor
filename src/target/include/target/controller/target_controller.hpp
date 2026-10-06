#pragma once

#include <beam/dto/beam_request.hpp>
#include <target/dto/detection_event.hpp>
#include <target/dto/track_event.hpp>

#include <optional>
#include <vector>

namespace target
{

class TargetController
{
public:
    std::optional<beam::BeamRequest> handleDetection(DetectionEvent detection);
    std::vector<TrackEvent> nextTrackEvents() const;
};

} // namespace target
