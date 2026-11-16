#pragma once

#include <beam/dto/request_beam_command.hpp>
#include <pose/dto/set_enu_geo_transform_command.hpp>
#include <target/config/target_config.hpp>
#include <target/dto/detection_event.hpp>
#include <target/dto/track_event.hpp>
#include <spdlog/logger.h>

#include <chrono>
#include <cstddef>
#include <memory>
#include <optional>
#include <vector>

namespace target
{

class DetectionHandler;
class DetectionEventLogger;
class TrackExpirationHandler;
class TrackRepository;
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

    // 응답 대기 한도를 넘긴 트랙을 삭제한다. 탐지 결과가 오지 않아도 정리되도록 주기적으로 호출한다.
    void removeExpiredTracks(std::chrono::milliseconds currentTime);
    std::vector<TrackEvent> getTrackEvents() const;

    // 잠정-확정 트랙과 탐색 결과 기록을 모두 지우고 삭제한 트랙 수를 반환한다.
    // 트랙 ID 순번은 유지하므로 클리어 전에 요청한 빔의 결과가 늦게 와도 무시된다.
    std::size_t clearTracks();

private:
    std::shared_ptr<TrackRepository> _trackRepository;
    std::shared_ptr<DetectionHandler> _detectionHandler;
    std::shared_ptr<TrackExpirationHandler> _trackExpirationHandler;
    std::shared_ptr<TrackReporter> _trackReporter;
    std::shared_ptr<DetectionEventLogger> _detectionLogger;
    std::shared_ptr<spdlog::logger> _messageLogger;
};

} // namespace target
