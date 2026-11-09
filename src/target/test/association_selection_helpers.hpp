#pragma once

#include "domain/algorithm/nds_algorithm.hpp"

#include "domain/selector/tentative_selector.hpp"
#include "domain/selector/track_selector.hpp"

namespace target::test_support
{

using namespace math::literals;

inline std::optional<std::size_t> selectTracking(
    const Observation& observation,
    const ConfirmedTrack& track,
    AssociationConfig config = AssociationConfig(1000_mps, 30_mps, 11.345),
    MeasurementConfig measurement = MeasurementConfig(7_m)
) {
    const TrackSelector selector(config, std::make_unique<NdsAlgorithm>(measurement));
    return selector.selectObservation(track, observation.lastObservationTime(), {observation});
}

template <typename TrackType>
bool isDuplicate(
    const Observation& observation,
    const TrackType& track,
    AssociationConfig config = AssociationConfig(1000_mps, 30_mps, 11.345),
    MeasurementConfig measurement = MeasurementConfig(7_m)
) {
    const std::list<Track> tracks{track};
    const TentativeSelector selector(config, std::make_unique<NdsAlgorithm>(measurement));
    return !selector.selectNewCandidate({observation}, tracks);
}

} // namespace target::test_support
