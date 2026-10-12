#include "track_selector.hpp"

#include "repository/target_repository.hpp"

namespace target
{

TrackSelector::TrackSelector(const TargetRepository& repository)
    : _targetRepository(repository)
{
}

std::optional<std::size_t> TrackSelector::selectObservation(
    const DetectionEvent& detection) const
{
    const Target* requestedTarget = _targetRepository.find(detection.requestId);
    if (!requestedTarget)
    {
        return std::nullopt;
    }

    const Target& target = *requestedTarget;
    const bool tentative = target.state() == TargetState::TENTATIVE;
    const auto expectedBeamType = tentative
        ? beam::BeamCommand::BeamType::CONFIRMATION
        : beam::BeamCommand::BeamType::TRACKING;
    if (detection.beamType != expectedBeamType || detection.requestId != target.id()
        || detection.transmitTime <= target.lastResultTime())
    {
        return std::nullopt;
    }

    const auto prediction = tentative ? std::nullopt : target.predict(detection.transmitTime);
    for (std::size_t index = 0; index < detection.plots.size(); ++index)
    {
        const DetectionEvent::Plot& plot = detection.plots[index];
        if (tentative)
        {
            if (target.canConfirm(plot, detection.transmitTime))
            {
                return index;
            }
            continue;
        }

        // Temporary exact prediction match; NDS association is not implemented.
        if (prediction && _matcher.matchesPosition(plot, *prediction))
        {
            return index;
        }
    }

    return std::nullopt;
}

} // namespace target
