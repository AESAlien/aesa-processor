#include "tentative_selector.hpp"

#include "service/detection_matcher.hpp"
#include "repository/track_repository.hpp"

namespace target
{

TentativeSelector::TentativeSelector(
    const TrackRepository& repository,
    const DetectionMatcher& matcher
) : _trackRepository(repository),
    _matcher(matcher)
{
}

std::optional<std::size_t> TentativeSelector::selectNewCandidate(
    const std::vector<Observation>& observations
) const {
    for (std::size_t index = 0; index < observations.size(); ++index)
    {
        const Observation& observation = observations[index];
        bool associated = false;
        for (const auto& target : _trackRepository.targets())
        {
            const auto score = std::visit(
                [&](const auto& registeredTarget)
                {
                    return _matcher.matchesPosition(observation, registeredTarget);
                },
                target
            );
            if (score)
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
