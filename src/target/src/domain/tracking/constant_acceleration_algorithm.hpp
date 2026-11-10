#pragma once

#include "domain/tracking/tracking_algorithm.hpp"
#include "domain/tracking/position_uncertainty.hpp"

#include <target/config/measurement_config.hpp>
#include <target/config/tracking_config.hpp>

namespace target
{

namespace test_support
{
class KalmanStateAccess;
}

// 위치, 속도, 가속도의 XYZ 성분을 추정하는 9차원 등가속도 칼만 필터다.
class ConstantAccelerationAlgorithm final : public ITrackingAlgorithm, public IPositionUncertainty
{
public:
    ConstantAccelerationAlgorithm(TrackingConfig config, MeasurementConfig measurementConfig);

    std::optional<TrackingResult> initialize(
        const Observation& first,
        const Observation& second
    ) const override;

    std::optional<TrackingResult> update(
        const ConfirmedTrack& current,
        const Observation& observation
    ) const override;

    std::optional<TrackPrediction> predict(
        const ConfirmedTrack& current,
        std::chrono::milliseconds time
    ) const override;

    std::optional<TrackPrediction> predictWithPositionCovariance(
        const ConfirmedTrack& current,
        std::chrono::milliseconds time,
        std::optional<math::SquareMatrix>& covariance
    ) const override;

    std::optional<math::SquareMatrix> observedPositionCovariance(const ConfirmedTrack& current) const override;

private:
    friend class test_support::KalmanStateAccess;

    static void storeCovariance(std::any& state, math::SquareMatrix covariance);
    static const math::SquareMatrix* stateCovariance(const std::any& state) noexcept;

    std::optional<TrackPrediction> predictPosition(
        const ConfirmedTrack& current,
        std::chrono::milliseconds time,
        std::optional<math::SquareMatrix>* covariance
    ) const;

    const TrackingConfig _config;
    const MeasurementConfig _measurementConfig;
};

} // namespace target
