#pragma once

#include <beam/dto/request_beam_command.hpp>
#include <pose/dto/set_enu_geo_transform_command.hpp>
#include <target/config/target_config.hpp>
#include <target/dto/detection_event.hpp>
#include <target/dto/track_event.hpp>
#include <spdlog/logger.h>

#include <chrono>
#include <memory>
#include <optional>
#include <vector>

namespace target
{

class DetectionHandler;
class DetectionEventLogger;
class TrackExpirationHandler;
class TrackReporter;

// 복사본은 원본과 처리기 및 트랙 저장소를 공유한다.
// 공유 상태에 접근하는 호출은 호출자가 직렬화해야 한다.
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
    // ENU 좌표 변환을 설정하기 전에는 빈 목록을 반환한다.
    std::vector<TrackEvent> getTrackEvents() const;

private:
    std::shared_ptr<DetectionHandler> _detectionHandler;
    std::shared_ptr<TrackExpirationHandler> _trackExpirationHandler;
    std::shared_ptr<TrackReporter> _trackReporter;
    std::shared_ptr<DetectionEventLogger> _detectionLogger;
    std::shared_ptr<spdlog::logger> _messageLogger;
};

} // namespace target
