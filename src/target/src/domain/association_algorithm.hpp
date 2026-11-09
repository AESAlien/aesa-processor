#pragma once

#include "track_prediction.hpp"

#include <math/bounded/distance.hpp>
#include <math/geometry/spherical_position.hpp>
#include <math/linear_algebra/vector.hpp>

#include <optional>

namespace target
{

class ConfirmedTrack;

// 선택기마다 별도 인스턴스를 사용한다. 준비를 다시 하면 기존 상태가 교체되며, 실패하면 비워진다.
// 준비부터 점수 계산까지 한 관측 묶음의 처리를 끝낸 뒤 다음 묶음을 처리해야 한다.
class IAssociationAlgorithm
{
public:
    virtual ~IAssociationAlgorithm() = default;

    virtual bool prepareMeasurements() = 0;
    virtual std::optional<TrackPrediction> predictAndPrepare(
        const ConfirmedTrack& track,
        std::chrono::milliseconds time
    ) = 0;
    virtual bool prepareFromLastObservation(const ConfirmedTrack& track) = 0;

    // 점수가 낮을수록 연관성이 높다. 준비 상태가 없거나 계산할 수 없으면 nullopt를 반환한다.
    virtual std::optional<double> calculate(const math::Vector& residual) const = 0;
    virtual std::optional<double> calculate(
        const math::SphericalPosition& observation,
        const math::SphericalPosition& reference,
        std::optional<math::Distance> maximumTravel = std::nullopt
    ) const = 0;
};

} // namespace target
