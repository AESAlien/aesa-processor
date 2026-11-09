#pragma once

#include "domain/observation.hpp"
#include "domain/track.hpp"
#include "domain/association_algorithm.hpp"

#include <target/config/association_config.hpp>

#include <list>
#include <optional>
#include <memory>
#include <vector>

namespace target
{

class TentativeTrack;
class ConfirmedTrack;

// 연관 점수 계산에 쓰는 준비 상태를 공유하므로 한 인스턴스에 대한 호출은 겹치면 안 된다.
class TentativeSelector
{
public:
    TentativeSelector(
        AssociationConfig config,
        std::unique_ptr<IAssociationAlgorithm> algorithm
    );

    std::optional<Observation> selectNewCandidate(
        const std::vector<Observation>& observations,
        const std::list<Track>& tracks
    ) const;

private:
    std::optional<double> matchingPositionScore(const Observation& observation, const TentativeTrack& other) const;
    std::optional<double> matchingPositionScore(const Observation& observation, const ConfirmedTrack& other) const;
    std::optional<double> positionScore(
        const Observation& observation,
        const math::SphericalPosition& referencePosition,
        std::chrono::milliseconds referenceTime,
        bool compensateMotion
    ) const;

    const AssociationConfig _config;
    const std::unique_ptr<IAssociationAlgorithm> _algorithm;
};

} // namespace target
