#pragma once

#include "domain/observation.hpp"
#include "domain/track.hpp"
#include "domain/association/association_algorithm.hpp"

#include <target/config/association_config.hpp>

#include <list>
#include <optional>
#include <memory>
#include <vector>

namespace target
{

class TentativeTrack;
class ConfirmedTrack;

class TentativeSelector
{
public:
    TentativeSelector(
        AssociationConfig config,
        std::unique_ptr<const IAssociationAlgorithm> algorithm
    );

    std::optional<Observation> selectNewCandidate(
        const std::vector<Observation>& observations,
        const std::list<Track>& tracks
    ) const;

private:
    std::optional<double> matchingPositionScore(const Observation& observation, const TentativeTrack& other) const;
    std::optional<double> matchingPositionScore(const Observation& observation, const ConfirmedTrack& other) const;
    std::optional<double> positionScore(
        const IAssociationCalculation& calculation,
        const Observation& observation,
        const math::SphericalPosition& referencePosition,
        std::chrono::milliseconds referenceTime,
        bool compensateMotion
    ) const;

    const AssociationConfig _config;
    const std::unique_ptr<const IAssociationAlgorithm> _algorithm;
};

} // namespace target
