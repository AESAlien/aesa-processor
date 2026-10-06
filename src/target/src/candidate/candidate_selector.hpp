#pragma once

#include <target/dto/detection_event.hpp>

namespace target
{

class TrackRepository;

class CandidateSelector
{
public:
    bool isNewCandidate(
        const DetectionEvent& detection,
        const TrackRepository& trackRepository) const;
};

} // namespace target
