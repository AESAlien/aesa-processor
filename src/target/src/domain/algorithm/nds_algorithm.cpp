#include "nds_algorithm.hpp"

#include "nds_calculation.hpp"
#include "domain/position_uncertainty.hpp"

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

bool NdsAlgorithm::prepareMeasurements()
{
    _calculation.reset();
    if (_measurementDifferenceCovariance)
    {
        _calculation = NdsCalculation::from(*_measurementDifferenceCovariance);
    }
    return _calculation.has_value();
}

std::optional<TrackPrediction> NdsAlgorithm::predictAndPrepare(
    const ConfirmedTrack& track,
    std::chrono::milliseconds time
) {
    // 준비에 실패한 경우 이전 트랙의 공분산으로 점수를 계산하지 않도록 먼저 비운다.
    _calculation.reset();
    std::optional<math::SquareMatrix> covariance;
    auto prediction = IPositionUncertainty::predict(track, time, covariance);
    if (!prediction || !prepareCovariance(covariance))
    {
        return std::nullopt;
    }
    return prediction;
}

bool NdsAlgorithm::prepareFromLastObservation(const ConfirmedTrack& track)
{
    _calculation.reset();
    // 마지막 관측보다 오래된 관측과 비교할 때는 저장된 추정 위치의 공분산을 기준으로 쓴다.
    return prepareCovariance(IPositionUncertainty::atLastObservation(track));
}

bool NdsAlgorithm::prepareCovariance(const std::optional<math::SquareMatrix>& covariance)
{
    if (!covariance)
    {
        // 기준 위치의 불확실성도 측정 잡음으로 근사하므로 잔차 공분산에 2R을 쓴다.
        return prepareMeasurements();
    }
    if (!_measurementCovariance)
    {
        return false;
    }
    try
    {
        // 기준 위치 오차와 새 관측 잡음이 독립이면 잔차 공분산은 Ppos + R이다.
        // 같은 기준에 대한 여러 후보의 점수 계산에 이 분해를 재사용한다.
        _calculation = NdsCalculation::from(*covariance + *_measurementCovariance);
        return _calculation.has_value();
    }
    catch (const std::invalid_argument&)
    {
        return false;
    }
}

std::optional<double> NdsAlgorithm::calculate(const math::Vector& residual) const
{
    return _calculation ? _calculation->calculate(residual) : std::nullopt;
}

std::optional<double> NdsAlgorithm::calculate(
    const math::SphericalPosition& observation,
    const math::SphericalPosition& reference,
    std::optional<math::Distance> maximumTravel
) const {
    return _calculation ? _calculation->calculate(observation, reference, maximumTravel) : std::nullopt;
}

} // namespace target
