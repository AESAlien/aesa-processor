#pragma once

#include "domain/detection_result.hpp"
#include "domain/track_factory.hpp"

#include <beam/dto/request_beam_command.hpp>
#include <target/config/track_lifecycle_config.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <unordered_map>

namespace target
{

class TrackRepository;
class TentativeSelector;
class TrackSelector;

class DetectionHandler
{
public:
    DetectionHandler(
        TrackLifecycleConfig config,
        std::shared_ptr<TrackRepository> repository,
        TrackFactory trackFactory,
        std::unique_ptr<const TentativeSelector> tentativeSelector,
        std::unique_ptr<const TrackSelector> trackSelector
    );
    ~DetectionHandler();

    std::optional<beam::RequestBeamCommand> handle(const DetectionResult& result);

    // 탐색 빔별 마지막 결과 시각을 모두 지운다. 운용을 다시 시작할 때 이전 기록을 남기지 않는다.
    void clearSearchHistory();

private:
    std::optional<beam::RequestBeamCommand> handle(const SearchDetectionResult& result);
    std::optional<beam::RequestBeamCommand> handle(const ConfirmationDetectionResult& result);
    std::optional<beam::RequestBeamCommand> handle(const TrackingDetectionResult& result);

    const TrackLifecycleConfig _config;
    std::shared_ptr<TrackRepository> _trackRepository;
    TrackFactory _trackFactory;
    std::unique_ptr<const TentativeSelector> _tentativeSelector;
    std::unique_ptr<const TrackSelector> _trackSelector;
    std::unordered_map<std::uint32_t, std::chrono::milliseconds> _lastSearchResultTimes;
};

} // namespace target
