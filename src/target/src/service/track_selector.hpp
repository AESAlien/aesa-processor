#pragma once

#include "domain/observation.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace target
{

class TrackRepository;
class DetectionMatcher;
class Track;
class TentativeTrack;

// Select the eligible observation with minimum NDS without modifying the requested target.
class TrackSelector
{
public:
    TrackSelector(
        const TrackRepository& repository,
        const DetectionMatcher& matcher
    );

    std::optional<std::size_t> selectObservation(
        std::uint32_t requestId,
        std::chrono::milliseconds resultTime,
        const std::vector<Observation>& observations
    ) const;

    std::optional<std::size_t> selectConfirmation(
        std::uint32_t requestId,
        std::chrono::milliseconds resultTime,
        const std::vector<Observation>& observations
    ) const;

    // References are used only during this call, before any repository mutation.
    std::optional<std::size_t> selectObservation(
        const Track& track,
        std::chrono::milliseconds resultTime,
        const std::vector<Observation>& observations
    ) const;

    std::optional<std::size_t> selectConfirmation(
        const TentativeTrack& tentative,
        std::chrono::milliseconds resultTime,
        const std::vector<Observation>& observations
    ) const;

private:
    // The injected repository and matcher must outlive this selector.
    const TrackRepository& _trackRepository;
    const DetectionMatcher& _matcher;
};

} // namespace target
