#pragma once

#include "domain/target.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

namespace target
{

// Owns tentative and tracking targets under the same stable target ID.
class TargetRepository
{
public:
    explicit TargetRepository(std::shared_ptr<const ITrackingAlgorithm> trackingAlgorithm);

    std::uint32_t add(const DetectionEvent::Plot& initialPlot, std::chrono::milliseconds time);
    Target* find(std::uint32_t id);
    const Target* find(std::uint32_t id) const;
    bool remove(std::uint32_t id);
    const std::vector<Target>& targets() const;

    std::optional<std::chrono::milliseconds> lastSearchResultTime(std::uint32_t gridId) const;
    void markSearchResultProcessed(std::uint32_t gridId, std::chrono::milliseconds time);

private:
    // The shared strategy retains no target state; each Target owns its state.
    std::shared_ptr<const ITrackingAlgorithm> _trackingAlgorithm;
    std::vector<Target> _targets;
    std::uint64_t _nextTargetId = 1;
    std::unordered_map<std::uint32_t, std::chrono::milliseconds> _lastSearchResultTimes;
};

} // namespace target
