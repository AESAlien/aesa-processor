#include "tentative_selector.hpp"

#include "repository/target_repository.hpp"

namespace target
{

TentativeSelector::TentativeSelector(const TargetRepository& repository)
    : _targetRepository(repository)
{
}

std::optional<std::size_t> TentativeSelector::selectNewCandidate(
    const std::vector<DetectionEvent::Plot>& plots) const
{
    for (std::size_t index = 0; index < plots.size(); ++index)
    {
        const DetectionEvent::Plot& plot = plots[index];
        if (!_matcher.isValidPosition(plot))
        {
            continue;
        }

        bool associated = false;
        for (const Target& target : _targetRepository.targets())
        {
            if (_matcher.matchesPosition(plot, target.lastPlot()))
            {
                associated = true;
                break;
            }
        }

        if (!associated)
        {
            return index;
        }
    }

    return std::nullopt;
}

} // namespace target
