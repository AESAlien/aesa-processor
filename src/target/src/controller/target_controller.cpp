#include <target/controller/target_controller.hpp>

#include "domain/algorithm/constant_acceleration_algorithm.hpp"
#include "domain/algorithm/nds_algorithm.hpp"
#include "domain/factory/detection_result_factory.hpp"
#include "domain/factory/track_factory.hpp"
#include "dto/factory/track_event_factory.hpp"
#include "repository/track_repository.hpp"
#include "service/detection_handler.hpp"
#include "domain/selector/tentative_selector.hpp"
#include "domain/selector/track_selector.hpp"

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
    _tentativeSelector(
        std::make_shared<TentativeSelector>(
            config.association,
            std::make_unique<NdsAlgorithm>(config.measurement)
        )
    ),
    _trackSelector(
        std::make_shared<TrackSelector>(
            config.association,
            std::make_unique<NdsAlgorithm>(config.measurement)
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
        if (const auto* track = std::get_if<ConfirmedTrack>(&target))
        {
            events.push_back(TrackEventFactory::from(*track));
            if (events.size() == _config->maxReportedTrackCount())
            {
                break;
            }
        }
    }
    return events;
}

} // namespace target
