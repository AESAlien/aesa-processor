#pragma once

#include <beam/dto/beam_request.hpp>
#include <target/dto/detection_event.hpp>
#include <target/dto/track_event.hpp>

#include <memory>
#include <optional>
#include <vector>

namespace target
{

class TargetRepository;

class TargetController
{
public:
    TargetController();

    std::optional<beam::BeamRequest> handleDetection(DetectionEvent detection);
    std::vector<TrackEvent> nextTrackEvents() const;

private:
    std::shared_ptr<TargetRepository> _targetRepository;
};

} // namespace target
