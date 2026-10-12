#pragma once

#include "calculation/association/detection_matcher.hpp"

#include <cstddef>
#include <optional>

namespace target
{

class TargetRepository;

// Select one observation for the requested target without modifying its state.
class TrackSelector
{
public:
    explicit TrackSelector(const TargetRepository& repository);

    std::optional<std::size_t> selectObservation(
        const DetectionEvent& detection) const;

private:
    // The injected repository must outlive this selector.
    const TargetRepository& _targetRepository;
    DetectionMatcher _matcher{};
};

} // namespace target
