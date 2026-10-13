#include "tentative_selector.hpp"

#include "calculation/association/detection_matcher.hpp"
#include "repository/target_repository.hpp"

namespace target
{

TentativeSelector::TentativeSelector(
    const TargetRepository& repository,
    const DetectionMatcher& matcher)
    : _targetRepository(repository)
    , _matcher(matcher)
{
}

std::optional<std::size_t> TentativeSelector::selectNewCandidate(
    const std::vector<Target>& observations) const
{
    for (std::size_t index = 0; index < observations.size(); ++index)
    {
        const Target& observation = observations[index];
        if (observation.state() != TargetState::UNREGISTERED
            || !_matcher.isValidPosition(observation))
        {
            continue;
        }

        bool associated = false;
        for (const Target& target : _targetRepository.targets())
        {
            if (_matcher.matchesPosition(observation, target))
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
