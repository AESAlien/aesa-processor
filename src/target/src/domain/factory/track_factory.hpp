#pragma once

#include "domain/tentative_track.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>

namespace target
{

class TrackFactory
{
public:
    explicit TrackFactory(std::shared_ptr<const ITrackingAlgorithm> trackingAlgorithm);

    TentativeTrack from(std::uint32_t id, Observation observation) const;

    // The tentative target owns confirmation and delegates construction back here.
    static std::optional<Track> from(
        const TentativeTrack& tentative,
        const Observation& observation
    );

private:
    friend class TentativeTrack;

    static Track from(
        std::uint32_t id,
        std::chrono::milliseconds initialTime,
        TargetTrackingState state,
        std::shared_ptr<const ITrackingAlgorithm> trackingAlgorithm
    );

    std::shared_ptr<const ITrackingAlgorithm> _trackingAlgorithm;
};

} // namespace target
