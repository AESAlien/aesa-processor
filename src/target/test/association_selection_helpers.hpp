#pragma once

#include "domain/tracking/constant_velocity_algorithm.hpp"
#include "domain/association/nds_algorithm.hpp"
#include "domain/tracking/position_uncertainty.hpp"

#include "domain/association/tentative_selector.hpp"
#include "domain/association/track_selector.hpp"

#include <utility>

namespace target::test_support
{

using namespace math::literals;

class CovarianceAlgorithm final : public ITrackingAlgorithm, public IPositionUncertainty
{
public:
    explicit CovarianceAlgorithm(math::SquareMatrix covariance, double growthPerSecond = 0.0)
      : _covariance(std::move(covariance)), _growthPerSecond(growthPerSecond)
    {
    }

    std::optional<TrackingResult> initialize(const Observation& first, const Observation& second) const override
    {
        return _algorithm.initialize(first, second);
    }

    std::optional<TrackingResult> update(const ConfirmedTrack& current, const Observation& incoming) const override
    {
        return _algorithm.update(current, incoming);
    }

    std::optional<TrackPrediction> predict(const ConfirmedTrack& current, std::chrono::milliseconds time) const override
    {
        return _algorithm.predict(current, time);
    }

    std::optional<TrackPrediction> predictWithPositionCovariance(
        const ConfirmedTrack& current,
        std::chrono::milliseconds time,
        std::optional<math::SquareMatrix>& covariance
    ) const override {
        const auto prediction = predict(current, time);
        if (prediction)
        {
            const auto seconds = std::chrono::duration<double>(time - current.lastObservationTime()).count();
            covariance = _covariance * (1.0 + _growthPerSecond * seconds);
        }
        return prediction;
    }

    std::optional<math::SquareMatrix> observedPositionCovariance(const ConfirmedTrack&) const override
    {
        return _covariance;
    }

private:
    const math::SquareMatrix _covariance;
    const double _growthPerSecond;
    ConstantVelocityAlgorithm _algorithm;
};

inline std::optional<Observation> selectTracking(
    const Observation& observation,
    const ConfirmedTrack& track,
    AssociationConfig config = AssociationConfig(1000_mps, 30_mps, 11.345),
    MeasurementConfig measurement = MeasurementConfig(7_m)
) {
    const auto selector = TrackSelector(config, std::make_unique<NdsAlgorithm>(measurement));
    return selector.selectTrackingObservation(track, observation.observationTime(), {observation});
}

template <typename TrackType>
bool isDuplicate(
    const Observation& observation,
    const TrackType& track,
    AssociationConfig config = AssociationConfig(1000_mps, 30_mps, 11.345),
    MeasurementConfig measurement = MeasurementConfig(7_m)
) {
    const auto tracks = std::list<Track>{track};
    const auto selector = TentativeSelector(config, std::make_unique<NdsAlgorithm>(measurement));
    return !selector.selectNewCandidate({observation}, tracks);
}

} // namespace target::test_support
