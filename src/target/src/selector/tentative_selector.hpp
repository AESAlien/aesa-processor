#pragma once

#include "calculation/association/detection_matcher.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace target
{

class TargetRepository;

// Select at most one new tentative target from a search beam result.
class TentativeSelector
{
public:
    explicit TentativeSelector(const TargetRepository& repository);

    std::optional<std::size_t> selectNewCandidate(
        const std::vector<DetectionEvent::Plot>& plots) const;

private:
    // The injected repository must outlive this selector.
    const TargetRepository& _targetRepository;
    DetectionMatcher _matcher{};
};

} // namespace target
