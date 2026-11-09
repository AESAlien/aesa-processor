#pragma once

#include "domain/observation.hpp"
#include "domain/association_algorithm.hpp"

#include <target/config/association_config.hpp>

#include <chrono>
#include <cstddef>
#include <optional>
#include <memory>
#include <vector>

namespace target
{

class ConfirmedTrack;
class TentativeTrack;

// Select the eligible observation with minimum association score without modifying the requested target.
class TrackSelector
{
public:
    TrackSelector(
        AssociationConfig config,
        std::unique_ptr<IAssociationAlgorithm> algorithm
    );

    // The caller supplies the target; references are not retained after this call.
    std::optional<std::size_t> selectObservation(
        const ConfirmedTrack& track,
        std::chrono::milliseconds resultTime,
        const std::vector<Observation>& observations
    ) const;

    std::optional<std::size_t> selectConfirmation(
        const TentativeTrack& tentative,
        std::chrono::milliseconds resultTime,
        const std::vector<Observation>& observations
    ) const;

private:
    std::optional<double> confirmationScore(const Observation& observation, const TentativeTrack& tentative) const;

    // Owns mutable preparation state. Calls on one selector must not overlap.
    const AssociationConfig _config;
    const std::unique_ptr<IAssociationAlgorithm> _algorithm;
};

} // namespace target
