#include "track_selector.hpp"

#include "service/detection_matcher.hpp"
#include "repository/track_repository.hpp"

namespace target
{
namespace
{

template <typename TargetType, typename Match>
std::optional<std::size_t> selectBestObservation(
    const TargetType* target,
    std::chrono::milliseconds resultTime,
    const std::vector<Observation>& observations,
    const Match& match
) {
    if (!target || resultTime <= target->lastResultTime())
    {
        return std::nullopt;
    }

    std::optional<std::size_t> bestIndex;
    double bestScore = 0.0;
    for (std::size_t index = 0; index < observations.size(); ++index)
    {
        const auto& observation = observations[index];
        if (observation.lastObservationTime() != resultTime)
        {
            continue;
        }

        const auto score = match(observation, *target);
        // Keep the first observation when two eligible scores are equal.
        if (score && (!bestIndex || *score < bestScore))
        {
            bestIndex = index;
            bestScore = *score;
        }
    }
    return bestIndex;
}

} // namespace

TrackSelector::TrackSelector(
    const TrackRepository& repository,
    const DetectionMatcher& matcher
) : _trackRepository(repository),
    _matcher(matcher)
{
}

std::optional<std::size_t> TrackSelector::selectObservation(
    std::uint32_t requestId,
    std::chrono::milliseconds resultTime,
    const std::vector<Observation>& observations
) const {
    const auto* track = _trackRepository.findTrack(requestId);
    return track ? selectObservation(*track, resultTime, observations) : std::nullopt;
}

std::optional<std::size_t> TrackSelector::selectObservation(
    const Track& track,
    std::chrono::milliseconds resultTime,
    const std::vector<Observation>& observations
) const {
    bool prepared = false;
    std::optional<TrackingAssociation> association;
    // Empty batches and observations at other times must not trigger prediction.
    return selectBestObservation(
        &track,
        resultTime,
        observations,
        [&](const Observation& observation, const Track& current)
        {
            if (!prepared)
            {
                association = _matcher.prepareTracking(current, resultTime);
                prepared = true;
            }
            return association ? association->matches(observation) : std::nullopt;
        }
    );
}

std::optional<std::size_t> TrackSelector::selectConfirmation(
    std::uint32_t requestId,
    std::chrono::milliseconds resultTime,
    const std::vector<Observation>& observations
) const {
    const auto* tentative = _trackRepository.findTentative(requestId);
    return tentative ? selectConfirmation(*tentative, resultTime, observations) : std::nullopt;
}

std::optional<std::size_t> TrackSelector::selectConfirmation(
    const TentativeTrack& tentative,
    std::chrono::milliseconds resultTime,
    const std::vector<Observation>& observations
) const {
    return selectBestObservation(
        &tentative,
        resultTime,
        observations,
        [this](const Observation& observation, const TentativeTrack& current)
        {
            return _matcher.matchesConfirmation(observation, current);
        }
    );
}

} // namespace target
