#pragma once

#include "observation.hpp"
#include "track.hpp"

namespace target
{

// Confirmation delegates Track creation without mutating this first observation.
class TentativeTrack
{
public:
    TentativeTrack(
        std::uint32_t id,
        Observation observation,
        std::shared_ptr<const ITrackingAlgorithm> trackingAlgorithm
    );

    std::uint32_t id() const;
    const Observation& observation() const;
    std::chrono::milliseconds lastResultTime() const;

    std::optional<TargetTrackingState> initialize(const Observation& observation) const;
    std::optional<Track> confirm(const Observation& observation) const;

private:
    const std::uint32_t _id;
    const Observation _observation;
    const std::shared_ptr<const ITrackingAlgorithm> _trackingAlgorithm;
};

} // namespace target
