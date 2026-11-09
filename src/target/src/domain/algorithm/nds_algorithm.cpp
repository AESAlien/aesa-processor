#include "nds_algorithm.hpp"

#include "nds_calculation.hpp"
#include "domain/track_state.hpp"

#include <stdexcept>
#include <utility>

namespace target
{

std::optional<math::SquareMatrix> positionCovariance(const TargetPrediction& prediction)
{
    return prediction._positionCovariance;
}

namespace
{

std::optional<math::SquareMatrix> positionCovariance(const TrackState& state)
{
    const auto& covariance = state.errorCovariance();
    if (!covariance)
    {
        return std::nullopt;
    }

    math::SquareMatrix result(NdsCalculation::DIMENSION);
    for (std::size_t row = 0; row < NdsCalculation::DIMENSION; ++row)
    {
        for (std::size_t column = 0; column < NdsCalculation::DIMENSION; ++column)
        {
            result(row, column) = (*covariance)(row, column);
        }
    }
    return result;
}

std::optional<math::SquareMatrix> measurementCovariance(const MeasurementConfig& measurement, double count)
{
    try
    {
        const double sigma = measurement.standardDeviation().m();
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

bool NdsAlgorithm::preparePrediction(const TargetPrediction& prediction)
{
    _calculation.reset();
    const auto covariance = positionCovariance(prediction);
    if (!covariance)
    {
        return prepareMeasurements();
    }
    if (!_measurementCovariance)
    {
        return false;
    }
    try
    {
        _calculation = NdsCalculation::from(*covariance + *_measurementCovariance);
        return _calculation.has_value();
    }
    catch (const std::invalid_argument&)
    {
        return false;
    }
}

bool NdsAlgorithm::prepareState(const TrackState& state)
{
    _calculation.reset();
    const auto covariance = positionCovariance(state);
    if (!covariance)
    {
        return prepareMeasurements();
    }
    if (!_measurementCovariance)
    {
        return false;
    }
    try
    {
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
