#include "nds_algorithm.hpp"

#include "nds_calculation.hpp"
#include "domain/tracking/position_uncertainty.hpp"

#include <stdexcept>
#include <utility>

namespace target
{

namespace
{

std::optional<math::SquareMatrix> measurementCovariance(const MeasurementConfig& measurement, double count)
{
    try
    {
        const auto sigma = measurement.standardDeviation().m();
        // 세 축의 측정 오차는 서로 독립이며 같은 분산을 갖는다고 가정한다.
        return math::SquareMatrix::identity(NdsCalculation::DIMENSION) * (count * sigma * sigma);
    }
    catch (const std::invalid_argument&)
    {
        return std::nullopt;
    }
}

} // namespace

NdsAlgorithm::NdsAlgorithm(const MeasurementConfig& measurement)
  : _measurementCovariance(measurementCovariance(measurement, 1.0)),
    _measurementDifferenceCovariance(measurementCovariance(measurement, 2.0))
{
}

AssociationCalculation NdsAlgorithm::prepareMeasurements() const
{
    if (!_measurementDifferenceCovariance)
    {
        return nullptr;
    }
    auto calculation = NdsCalculation::from(*_measurementDifferenceCovariance);
    return calculation ? std::make_unique<const NdsCalculation>(std::move(*calculation)) : nullptr;
}

std::optional<AssociationPrediction> NdsAlgorithm::predictAndPrepare(
    const ConfirmedTrack& track,
    std::chrono::milliseconds time
) const {
    std::optional<math::SquareMatrix> covariance;
    auto prediction = IPositionUncertainty::predict(track, time, covariance);
    if (!prediction)
    {
        return std::nullopt;
    }
    auto calculation = prepareCovariance(covariance);
    if (!calculation)
    {
        return std::nullopt;
    }
    return AssociationPrediction{std::move(*prediction), std::move(calculation)};
}

AssociationCalculation NdsAlgorithm::prepareFromLastObservation(const ConfirmedTrack& track) const
{
    // 마지막 관측보다 오래된 관측에는 저장된 추정 위치의 공분산을 사용한다.
    return prepareCovariance(IPositionUncertainty::atLastObservation(track));
}

AssociationCalculation NdsAlgorithm::prepareCovariance(const std::optional<math::SquareMatrix>& covariance) const
{
    if (!covariance)
    {
        // 기준 위치의 불확실성도 측정 잡음으로 근사하므로 잔차 공분산에 2R을 쓴다.
        return prepareMeasurements();
    }
    if (!_measurementCovariance)
    {
        return nullptr;
    }
    try
    {
        // 기준 위치 오차와 새 관측 잡음이 독립이면 잔차 공분산은 Ppos + R이다.
        auto calculation = NdsCalculation::from(*covariance + *_measurementCovariance);
        return calculation ? std::make_unique<const NdsCalculation>(std::move(*calculation)) : nullptr;
    }
    catch (const std::invalid_argument&)
    {
        return nullptr;
    }
}

} // namespace target
