#pragma once

#include "domain/detection_result.hpp"
#include "domain/track.hpp"
#include "domain/track_factory.hpp"

#include <beam/dto/request_beam_command.hpp>
#include <target/config/track_management_config.hpp>

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
        TrackManagementConfig config,
        std::shared_ptr<TrackRepository> repository,
        TrackFactory trackFactory,
        std::unique_ptr<const TentativeSelector> tentativeSelector,
        std::unique_ptr<const TrackSelector> trackSelector
    );
    ~DetectionHandler();

    std::optional<beam::RequestBeamCommand> handle(const DetectionResult& result);

private:
    std::optional<beam::RequestBeamCommand> handle(const SearchDetectionResult& result);
    std::optional<beam::RequestBeamCommand> handle(const ConfirmationDetectionResult& result);
    std::optional<beam::RequestBeamCommand> handle(const TrackingDetectionResult& result);

    // 요청한 빔의 결과가 응답 대기 한도를 넘도록 오지 않은 트랙을 삭제한다.
    void removeUnansweredTracks(std::chrono::milliseconds currentTime);
    std::chrono::milliseconds expectedResultTime(const Track& track) const;

    const TrackManagementConfig _config;
    std::shared_ptr<TrackRepository> _trackRepository;
    const TrackFactory _trackFactory;
    std::unique_ptr<const TentativeSelector> _tentativeSelector;
    std::unique_ptr<const TrackSelector> _trackSelector;
    std::unordered_map<std::uint32_t, std::chrono::milliseconds> _lastSearchResultTimes;
};

} // namespace target
