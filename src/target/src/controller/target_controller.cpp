#include <target/controller/target_controller.hpp>

#include "algorithm/constant_acceleration_algorithm.hpp"
#include "domain/factory/detection_result_factory.hpp"
#include "domain/factory/track_factory.hpp"
#include "repository/track_repository.hpp"
#include "service/detection_matcher.hpp"
#include "service/detection_handler.hpp"
#include "service/tentative_selector.hpp"
#include "service/track_selector.hpp"

namespace target
{

TargetController::TargetController(const TargetConfig& config)
  : _config(std::make_shared<const TrackManagementConfig>(config.trackManagement)),
    _trackFactory(
        std::make_shared<TrackFactory>(
            std::make_shared<ConstantAccelerationAlgorithm>(config.tracking, config.measurement)
        )
    ),
    _trackRepository(std::make_shared<TrackRepository>()),
    _matcher(std::make_shared<DetectionMatcher>(config.association, config.measurement)),
    _tentativeSelector(
        std::make_shared<TentativeSelector>(
            *_trackRepository,
            *_matcher
        )
    ),
    _trackSelector(
        std::make_shared<TrackSelector>(
            *_trackRepository,
            *_matcher
        )
    ),
    _detectionHandler(
        std::make_shared<DetectionHandler>(
            *_config,
            *_trackRepository,
            *_trackFactory,
            *_tentativeSelector,
            *_trackSelector
        )
    )
{
}

std::optional<beam::RequestBeamCommand> TargetController::handleDetection(DetectionEvent detection)
{
    const auto result = DetectionResultFactory::from(detection);
    return _detectionHandler->handle(result);
}

std::vector<TrackEvent> TargetController::nextTrackEvents() const
{
    std::vector<TrackEvent> events;
    for (const auto& target : _trackRepository->targets())
    {
        if (const auto* track = std::get_if<Track>(&target))
        {
            events.push_back(track->toTrackEvent());
            if (events.size() == _config->maxReportedTrackCount())
            {
                break;
            }
        }
    }
    return events;
}

} // namespace target
