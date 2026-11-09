#pragma once

#include <beam/dto/request_beam_command.hpp>
#include <target/config/target_config.hpp>
#include <target/dto/detection_event.hpp>
#include <target/dto/track_event.hpp>

#include <memory>
#include <optional>
#include <vector>

namespace target
{

class TrackRepository;
class TrackFactory;
class DetectionHandler;
class TentativeSelector;
class TrackSelector;

class TargetController
{
public:
    explicit TargetController(const TargetConfig& config);

    std::optional<beam::RequestBeamCommand> handleDetection(DetectionEvent detection);
    std::vector<TrackEvent> nextTrackEvents() const;

private:
    std::shared_ptr<const TrackManagementConfig> _config;
    std::shared_ptr<const TrackFactory> _trackFactory;
    std::shared_ptr<TrackRepository> _trackRepository;
    // Selectors and the handler borrow objects shared by controller copies.
    std::shared_ptr<const TentativeSelector> _tentativeSelector;
    std::shared_ptr<const TrackSelector> _trackSelector;
    std::shared_ptr<DetectionHandler> _detectionHandler;
};

} // namespace target
