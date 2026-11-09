#pragma once

#include "domain/observation.hpp"
#include "domain/track.hpp"
#include "domain/association_algorithm.hpp"

#include <target/config/association_config.hpp>

#include <cstddef>
#include <list>
#include <optional>
#include <memory>
#include <vector>

namespace target
{

class TentativeTrack;
class ConfirmedTrack;

// Select at most one new tentative target from a search beam result.
class TentativeSelector
{
public:
    TentativeSelector(
        AssociationConfig config,
        std::unique_ptr<IAssociationAlgorithm> algorithm
    );

    // Inputs are read only during this call and are not retained.
    std::optional<std::size_t> selectNewCandidate(
        const std::vector<Observation>& observations,
        const std::list<Track>& tracks
    ) const;

private:
    std::optional<double> matchesPosition(const Observation& observation, const TentativeTrack& other) const;
    std::optional<double> matchesPosition(const Observation& observation, const ConfirmedTrack& other) const;
    std::optional<double> positionScore(
        const Observation& observation,
        const math::SphericalPosition& referencePosition,
        std::chrono::milliseconds referenceTime,
        bool compensateMotion
    ) const;

    // Owns mutable preparation state. Calls on one selector must not overlap.
    const AssociationConfig _config;
    const std::unique_ptr<IAssociationAlgorithm> _algorithm;
};

} // namespace target
