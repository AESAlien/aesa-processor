#pragma once

#include "domain/association_algorithm.hpp"
#include "nds_calculation.hpp"

#include <target/config/measurement_config.hpp>
#include <math/linear_algebra/square_matrix.hpp>

#include <optional>

namespace target
{

class NdsAlgorithm final : public IAssociationAlgorithm
{
public:
    explicit NdsAlgorithm(const MeasurementConfig& measurement);

    bool prepareMeasurements() override;
    std::optional<TrackPrediction> predictAndPrepare(
        const ConfirmedTrack& track,
        std::chrono::milliseconds time
    ) override;
    bool prepareFromLastObservation(const ConfirmedTrack& track) override;

    std::optional<double> calculate(const math::Vector& residual) const override;
    std::optional<double> calculate(
        const math::SphericalPosition& observation,
        const math::SphericalPosition& reference,
        std::optional<math::Distance> maximumTravel = std::nullopt
    ) const override;

private:
    bool prepareCovariance(const std::optional<math::SquareMatrix>& covariance);

    // R은 관측 하나의 공분산, 2R은 독립인 두 관측의 차에 대한 공분산이다.
    const std::optional<math::SquareMatrix> _measurementCovariance;
    const std::optional<math::SquareMatrix> _measurementDifferenceCovariance;
    std::optional<NdsCalculation> _calculation;
};

} // namespace target
