#include <target/controller/target_controller.hpp>
#include <target/dto/log/detection_event_logger.hpp>

#include "domain/tracking/constant_acceleration_algorithm.hpp"
#include "domain/association/nds_algorithm.hpp"
#include "domain/factory/detection_result_factory.hpp"
#include "domain/track_factory.hpp"
#include "repository/track_repository.hpp"
#include "service/detection_handler.hpp"
#include "service/track_reporter.hpp"
#include "domain/association/tentative_selector.hpp"
#include "domain/association/track_selector.hpp"

#include <utility>

namespace target
{

TargetController::TargetController(
    const TargetConfig& config,
    std::shared_ptr<DetectionEventLogger> detectionLogger,
    std::shared_ptr<spdlog::logger> messageLogger
) : _detectionLogger(std::move(detectionLogger)),
    _messageLogger(std::move(messageLogger))
{
    const auto repository = std::make_shared<TrackRepository>();

    auto trackingAlgorithm = std::make_shared<ConstantAccelerationAlgorithm>(
        config.tracking,
        config.measurement
    );
    auto trackFactory = TrackFactory(std::move(trackingAlgorithm));

    auto tentativeAssociationAlgorithm = std::make_unique<NdsAlgorithm>(config.measurement);
    auto tentativeSelector = std::make_unique<TentativeSelector>(
        config.association,
        std::move(tentativeAssociationAlgorithm)
    );

    auto trackAssociationAlgorithm = std::make_unique<NdsAlgorithm>(config.measurement);
    auto trackSelector = std::make_unique<TrackSelector>(
        config.association,
        std::move(trackAssociationAlgorithm)
    );

    _detectionHandler = std::make_shared<DetectionHandler>(
        config.trackManagement,
        repository,
        std::move(trackFactory),
        std::move(tentativeSelector),
        std::move(trackSelector)
    );
    _trackReporter = std::make_shared<TrackReporter>(
        repository,
        config.trackManagement.maxReportedTrackCount()
    );
}

void TargetController::setEnuGeoTransform(pose::SetEnuGeoTransformCommand command)
{
    _trackReporter->setEnuGeoTransform(std::move(command.transform));
}

std::optional<beam::RequestBeamCommand> TargetController::handleDetection(DetectionEvent detection)
{
    // 검증에 실패한 탐지 이벤트도 남기기 위해 먼저 기록한다.
    if (_detectionLogger)
    {
        _detectionLogger->log(detection);
    }
    const auto result = DetectionResultFactory::from(detection);
    return _detectionHandler->handle(result);
}

std::vector<TrackEvent> TargetController::getTrackEvents() const
{
    return _trackReporter->getTrackEvents();
}

} // namespace target
