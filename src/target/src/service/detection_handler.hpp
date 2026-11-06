#pragma once

#include "domain/detection_result.hpp"
#include "track_id_generator.hpp"

#include <beam/dto/request_beam_command.hpp>
#include <target/config/track_management_config.hpp>

#include <chrono>
#include <cstdint>
#include <optional>
#include <unordered_map>

namespace target
{

class TrackRepository;
class TrackFactory;
class TentativeSelector;
class TrackSelector;

// Applies validated beam results to the track lifecycle and schedules the next request.
class DetectionHandler
{
public:
    // The repository, factory and selectors must outlive this handler.
    DetectionHandler(
        TrackManagementConfig config,
        TrackRepository& repository,
        const TrackFactory& trackFactory,
        const TentativeSelector& tentativeSelector,
        const TrackSelector& trackSelector
    );

    std::optional<beam::RequestBeamCommand> handle(const DetectionResult& result);

private:
    std::optional<beam::RequestBeamCommand> handle(const SearchDetectionResult& result);
    std::optional<beam::RequestBeamCommand> handle(const ConfirmationDetectionResult& result);
    std::optional<beam::RequestBeamCommand> handle(const TrackingDetectionResult& result);

    const TrackManagementConfig _config;
    TrackRepository& _trackRepository;
    const TrackFactory& _trackFactory;
    const TentativeSelector& _tentativeSelector;
    const TrackSelector& _trackSelector;
    TrackIdGenerator _trackIdGenerator;
    std::unordered_map<std::uint32_t, std::chrono::milliseconds> _lastSearchResultTimes;
};

} // namespace target
