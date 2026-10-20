#include "target_repository.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

namespace aesa_processor::target
{

std::uint32_t TargetRepository::nextTargetId() const
{
    // The current console contract must represent the same stable ID.
    if (_nextTargetId > std::numeric_limits<std::uint16_t>::max())
    {
        throw std::overflow_error("Target ID exceeds the console ID range");
    }

    return static_cast<std::uint32_t>(_nextTargetId);
}

std::uint32_t TargetRepository::add(Target target)
{
    if (target.id() != 0 || target.state() != TargetState::UNREGISTERED)
    {
        throw std::invalid_argument("Only an unregistered Target can be added");
    }

    const auto id = nextTargetId();
    target.registerAs(id);
    // Consume the ID only after storing the registered copy succeeds.
    _targets.push_back(std::move(target));
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

    // Geometry values are immutable, so Target cannot be move-assigned by vector::erase.
    std::vector<Target> remainingTargets;
    remainingTargets.reserve(_targets.size() - 1);
    for (auto current = _targets.begin(); current != _targets.end(); ++current)
    {
        if (current != existing)
        {
            remainingTargets.push_back(std::move(*current));
        }
    }
    _targets.swap(remainingTargets);
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

} // namespace aesa_processor::target
