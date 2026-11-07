#pragma once

#include "domain/tentative_track.hpp"

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

    // The tentative target owns initialization and confirmed-track construction.
    static std::optional<ConfirmedTrack> from(
        const TentativeTrack& tentative,
        const Observation& observation
    );

private:
    std::shared_ptr<const ITrackingAlgorithm> _trackingAlgorithm;
};

} // namespace target
