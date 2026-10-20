#pragma once

#include "domain/target.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace aesa_processor::target
{

class TargetRepository;
class DetectionMatcher;

// Select at most one new tentative target from a search beam result.
class TentativeSelector
{
public:
    TentativeSelector(
        const TargetRepository& repository,
        const DetectionMatcher& matcher);

    std::optional<std::size_t> selectNewCandidate(
        const std::vector<Target>& observations) const;

private:
    // The injected repository and matcher must outlive this selector.
    const TargetRepository& _targetRepository;
    const DetectionMatcher& _matcher;
};

} // namespace aesa_processor::target
