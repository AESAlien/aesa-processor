#pragma once

#include "domain/association_algorithm.hpp"
#include "nds_calculation.hpp"

#include <target/config/measurement_config.hpp>
#include <math/linear_algebra/square_matrix.hpp>

#include <optional>

namespace target
{

// Owns measurement covariance R and 2R, shared across targets and timestamps.
// Replaces its internal factorization on preparation; failure clears the current state.
class NdsAlgorithm final : public IAssociationAlgorithm
{
public:
    explicit NdsAlgorithm(const MeasurementConfig& measurement);

    bool prepareMeasurements() override;
    bool preparePrediction(const TargetPrediction& prediction) override;
    bool prepareState(const TrackState& state) override;

    std::optional<double> calculate(const math::Vector& residual) const override;
    std::optional<double> calculate(
        const math::SphericalPosition& observation,
        const math::SphericalPosition& reference,
        std::optional<math::Length> maximumTravel = std::nullopt
    ) const override;

private:
    // Unrepresentable measurement covariance makes preparation fail.
    const std::optional<math::SquareMatrix> _measurementCovariance;
    const std::optional<math::SquareMatrix> _measurementDifferenceCovariance;
    std::optional<NdsCalculation> _calculation;
};

} // namespace target
