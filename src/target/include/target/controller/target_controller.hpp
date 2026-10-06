#pragma once

#include <beam/dto/beam_request.hpp>
#include <target/dto/detection_event.hpp>
#include <target/dto/track_event.hpp>

#include <memory>
#include <optional>
#include <vector>

namespace target
{

class TrackRepository;

class TargetController
{
public:
    TargetController();
    explicit TargetController(std::shared_ptr<TrackRepository> trackRepository);

    std::optional<beam::BeamRequest> handleDetection(DetectionEvent detection);
    std::vector<TrackEvent> nextTrackEvents() const;

private:
    std::shared_ptr<TrackRepository> _trackRepository;
};

} // namespace target
