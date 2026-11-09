#pragma once

#include "domain/observation.hpp"
#include "domain/association_algorithm.hpp"

#include <target/config/association_config.hpp>

#include <chrono>
#include <optional>
#include <memory>
#include <vector>

namespace target
{

class ConfirmedTrack;
class TentativeTrack;

// 연관 점수 계산에 쓰는 준비 상태를 공유하므로 한 인스턴스에 대한 호출은 겹치면 안 된다.
class TrackSelector
{
public:
    TrackSelector(
        AssociationConfig config,
        std::unique_ptr<IAssociationAlgorithm> algorithm
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
    std::optional<double> confirmationScore(const Observation& observation, const TentativeTrack& tentative) const;

    const AssociationConfig _config;
    const std::unique_ptr<IAssociationAlgorithm> _algorithm;
};

} // namespace target
