#pragma once

#include "domain/association/association_algorithm.hpp"

#include <target/config/measurement_config.hpp>
#include <math/linear_algebra/square_matrix.hpp>

#include <optional>

namespace target
{

class NdsAlgorithm final : public IAssociationAlgorithm
{
public:
    explicit NdsAlgorithm(const MeasurementConfig& measurement);

    AssociationCalculation prepareMeasurements() const override;
    std::optional<AssociationPrediction> predictAndPrepare(
        const ConfirmedTrack& track,
        std::chrono::milliseconds time
    ) const override;
    AssociationCalculation prepareFromLastObservation(const ConfirmedTrack& track) const override;

private:
    AssociationCalculation prepareCovariance(const std::optional<math::SquareMatrix>& covariance) const;

    // R은 관측 하나의 공분산, 2R은 독립인 두 관측의 차에 대한 공분산이다.
    const std::optional<math::SquareMatrix> _measurementCovariance;
    const std::optional<math::SquareMatrix> _measurementDifferenceCovariance;
};

} // namespace target
