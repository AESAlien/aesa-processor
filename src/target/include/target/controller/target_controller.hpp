#pragma once

#include <beam/dto/request_beam_command.hpp>
#include <pose/dto/set_enu_geo_transform_command.hpp>
#include <target/config/target_config.hpp>
#include <target/dto/detection_event.hpp>
#include <target/dto/track_event.hpp>
#include <spdlog/logger.h>

#include <memory>
#include <optional>
#include <vector>

namespace target
{

class DetectionHandler;
class DetectionEventLogger;
class TrackReporter;

class TargetController
{
public:
    explicit TargetController(
        const TargetConfig& config,
        std::shared_ptr<DetectionEventLogger> detectionLogger = {},
        std::shared_ptr<spdlog::logger> messageLogger = {}
    );

    void setEnuGeoTransform(pose::SetEnuGeoTransformCommand command);
    std::optional<beam::RequestBeamCommand> handleDetection(DetectionEvent detection);
    std::vector<TrackEvent> getTrackEvents() const;

private:
    std::shared_ptr<DetectionHandler> _detectionHandler;
    std::shared_ptr<TrackReporter> _trackReporter;
    std::shared_ptr<DetectionEventLogger> _detectionLogger;
    std::shared_ptr<spdlog::logger> _messageLogger;
};

} // namespace target
