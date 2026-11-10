#pragma once

#include "domain/observation.hpp"
#include "domain/association/association_algorithm.hpp"

#include <target/config/association_config.hpp>

#include <chrono>
#include <optional>
#include <memory>
#include <vector>

namespace target
{

class ConfirmedTrack;
class TentativeTrack;

class TrackSelector
{
public:
    TrackSelector(
        AssociationConfig config,
        std::unique_ptr<const IAssociationAlgorithm> algorithm
    );

    std::optional<Observation> selectTrackingObservation(
        const ConfirmedTrack& confirmedTrack,
        std::chrono::milliseconds resultTime,
        const std::vector<Observation>& observations
    ) const;

    std::optional<Observation> selectConfirmationObservation(
        const TentativeTrack& tentative,
        std::chrono::milliseconds resultTime,
        const std::vector<Observation>& observations
    ) const;

private:
    std::optional<double> confirmationScore(
        const IAssociationCalculation& calculation,
        const Observation& observation,
        const TentativeTrack& tentative
    ) const;

    const AssociationConfig _config;
    const std::unique_ptr<const IAssociationAlgorithm> _algorithm;
};

} // namespace target
