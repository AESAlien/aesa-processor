#pragma once

#include "domain/observation.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace target
{

class TrackRepository;
class DetectionMatcher;

// Select at most one new tentative target from a search beam result.
class TentativeSelector
{
public:
    TentativeSelector(
        const TrackRepository& repository,
        const DetectionMatcher& matcher
    );

    std::optional<std::size_t> selectNewCandidate(
        const std::vector<Observation>& observations
    ) const;

private:
    // The injected repository and matcher must outlive this selector.
    const TrackRepository& _trackRepository;
    const DetectionMatcher& _matcher;
};

} // namespace target
