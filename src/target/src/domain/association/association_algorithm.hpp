#pragma once

#include "association_calculation.hpp"
#include "domain/track_prediction.hpp"

#include <memory>
#include <optional>
#include <utility>

namespace target
{

class ConfirmedTrack;

using AssociationCalculation = std::unique_ptr<const IAssociationCalculation>;
// 성공한 예측은 null이 아닌 계산 객체와 함께 반환한다.
using AssociationPrediction = std::pair<TrackPrediction, AssociationCalculation>;

// 준비 결과는 각각 독립적이며 알고리즘의 후속 호출에 영향을 받지 않는다.
class IAssociationAlgorithm
{
public:
    virtual ~IAssociationAlgorithm() = default;

    virtual AssociationCalculation prepareMeasurements() const = 0;
    virtual std::optional<AssociationPrediction> predictAndPrepare(
        const ConfirmedTrack& track,
        std::chrono::milliseconds time
    ) const = 0;
    virtual AssociationCalculation prepareFromLastObservation(const ConfirmedTrack& track) const = 0;
};

} // namespace target
