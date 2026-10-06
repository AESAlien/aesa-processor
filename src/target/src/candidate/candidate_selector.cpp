#include "candidate_selector.hpp"

#include "repository/track_repository.hpp"

namespace target
{

bool CandidateSelector::isNewCandidate(
    const DetectionEvent& detection,
    const TrackRepository& trackRepository) const
{
    for (const auto& track : trackRepository.tracks())
    {
        // Temporary exact-value comparison in the same antenna frame.
        // Replace this condition with association math in a later increment.
        if (detection.slantRange.m() == track.slantRange.m()
            && detection.azimuth.deg() == track.azimuth.deg()
            && detection.elevation.deg() == track.elevation.deg())
        {
            return false;
        }
    }
    return true;
}

} // namespace target
