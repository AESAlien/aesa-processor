#pragma once

#include "domain/tentative_track.hpp"

#include <cstdint>
#include <memory>

namespace target
{

class TrackFactory
{
public:
    explicit TrackFactory(std::shared_ptr<const ITrackingAlgorithm> trackingAlgorithm);

    TentativeTrack createTentative(std::uint32_t id, Observation observation) const;

private:
    std::shared_ptr<const ITrackingAlgorithm> _trackingAlgorithm;
};

} // namespace target
