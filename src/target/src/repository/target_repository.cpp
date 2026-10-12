#include "target_repository.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

namespace target
{

TargetRepository::TargetRepository(std::shared_ptr<const ITrackingAlgorithm> trackingAlgorithm)
    : _trackingAlgorithm(std::move(trackingAlgorithm))
{
    if (!_trackingAlgorithm)
    {
        throw std::invalid_argument("Tracking algorithm must not be null");
    }
}

std::uint32_t TargetRepository::add(
    const DetectionEvent::Plot& initialPlot,
    std::chrono::milliseconds time)
{
    // TrackEvent's current console contract must represent the same stable ID.
    if (_nextTargetId > std::numeric_limits<std::uint16_t>::max())
    {
        throw std::overflow_error("Target ID exceeds the TrackEvent console ID range");
    }

    const auto id = static_cast<std::uint32_t>(_nextTargetId);
    _targets.emplace_back(id, initialPlot, time, _trackingAlgorithm);
    ++_nextTargetId;
    return id;
}

Target* TargetRepository::find(std::uint32_t id)
{
    const auto existing = std::find_if(_targets.begin(), _targets.end(),
        [id](const Target& target)
        {
            return target.id() == id;
        });
    return existing == _targets.end() ? nullptr : &*existing;
}

const Target* TargetRepository::find(std::uint32_t id) const
{
    const auto existing = std::find_if(_targets.begin(), _targets.end(),
        [id](const Target& target)
        {
            return target.id() == id;
        });
    return existing == _targets.end() ? nullptr : &*existing;
}

bool TargetRepository::remove(std::uint32_t id)
{
    const auto existing = std::find_if(_targets.begin(), _targets.end(),
        [id](const Target& target)
        {
            return target.id() == id;
        });
    if (existing == _targets.end())
    {
        return false;
    }

    _targets.erase(existing);
    return true;
}

const std::vector<Target>& TargetRepository::targets() const
{
    return _targets;
}

std::optional<std::chrono::milliseconds> TargetRepository::lastSearchResultTime(
    std::uint32_t gridId) const
{
    const auto result = _lastSearchResultTimes.find(gridId);
    if (result == _lastSearchResultTimes.end())
    {
        return std::nullopt;
    }

    return result->second;
}

void TargetRepository::markSearchResultProcessed(
    std::uint32_t gridId,
    std::chrono::milliseconds time)
{
    if (time.count() < 0)
    {
        throw std::invalid_argument("Search result time must not be negative");
    }

    const auto result = _lastSearchResultTimes.find(gridId);
    if (result == _lastSearchResultTimes.end())
    {
        _lastSearchResultTimes.emplace(gridId, time);
    }
    else if (time > result->second)
    {
        result->second = time;
    }
}

} // namespace target
