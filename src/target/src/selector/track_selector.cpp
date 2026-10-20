#include "track_selector.hpp"

#include "calculation/association/detection_matcher.hpp"
#include "repository/target_repository.hpp"

namespace aesa_processor::target
{

TrackSelector::TrackSelector(
    const TargetRepository& repository,
    const DetectionMatcher& matcher)
    : _targetRepository(repository)
    , _matcher(matcher)
{
}

std::optional<std::size_t> TrackSelector::selectObservation(
    std::uint32_t requestId,
    std::chrono::milliseconds resultTime,
    const std::vector<Target>& observations) const
{
    const Target* requestedTarget = _targetRepository.find(requestId);
    if (!requestedTarget || resultTime <= requestedTarget->lastResultTime())
    {
        return std::nullopt;
    }

    const Target& target = *requestedTarget;
    if (target.state() != TargetState::TENTATIVE && target.state() != TargetState::TRACKING)
    {
        return std::nullopt;
    }

    std::optional<std::size_t> bestIndex;
    double bestScore = 0.0;
    for (std::size_t index = 0; index < observations.size(); ++index)
    {
        const Target& observation = observations[index];
        if (observation.state() != TargetState::UNREGISTERED
            || observation.lastObservationTime() != resultTime)
        {
            continue;
        }

        const auto score = target.state() == TargetState::TENTATIVE
            ? _matcher.matchesConfirmation(observation, target)
            : _matcher.matchesTracking(observation, target);
        // Keep the first observation when two eligible scores are equal.
        if (score && (!bestIndex || *score < bestScore))
        {
            bestIndex = index;
            bestScore = *score;
        }
    }

    return bestIndex;
}

} // namespace aesa_processor::target
