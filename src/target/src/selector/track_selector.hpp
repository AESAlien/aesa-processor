#pragma once

#include "domain/target.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace target
{

class TargetRepository;
class DetectionMatcher;

// Select one observation for the requested target without modifying its state.
class TrackSelector
{
public:
    TrackSelector(
        const TargetRepository& repository,
        const DetectionMatcher& matcher);

    std::optional<std::size_t> selectObservation(
        std::uint32_t requestId,
        std::chrono::milliseconds resultTime,
        const std::vector<Target>& observations) const;

private:
    // The injected repository and matcher must outlive this selector.
    const TargetRepository& _targetRepository;
    const DetectionMatcher& _matcher;
};

} // namespace target
