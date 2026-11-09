#pragma once

#include "track_prediction.hpp"

#include <math/linear_algebra/square_matrix.hpp>

#include <optional>

namespace target
{

class ConfirmedTrack;

// 위치 불확실성을 추정하는 전략이 선택적으로 구현한다.
class IPositionUncertainty
{
public:
    virtual ~IPositionUncertainty() = default;

    // 공분산은 안테나 직교 좌표 [px, py, pz] 순서이며 단위는 m^2다.
    virtual std::optional<TrackPrediction> predictWithPositionCovariance(
        const ConfirmedTrack& track,
        std::chrono::milliseconds time,
        std::optional<math::SquareMatrix>& covariance
    ) const = 0;

    virtual std::optional<math::SquareMatrix> observedPositionCovariance(const ConfirmedTrack& track) const = 0;

    static std::optional<TrackPrediction> predict(
        const ConfirmedTrack& track,
        std::chrono::milliseconds time,
        std::optional<math::SquareMatrix>& covariance
    );

    static std::optional<math::SquareMatrix> atLastObservation(const ConfirmedTrack& track);
};

} // namespace target
