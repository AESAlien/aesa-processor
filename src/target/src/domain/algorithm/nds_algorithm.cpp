#include "nds_algorithm.hpp"

#include "nds_calculation.hpp"
#include "domain/track_state.hpp"

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
    if (!prediction.positionCovariance())
    {
        return prepareMeasurements();
    }
    if (!_measurementCovariance)
    {
        return false;
    }
    try
    {
        _calculation = NdsCalculation::from(*prediction.positionCovariance() + *_measurementCovariance);
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
    if (!state.errorCovariance())
    {
        return prepareMeasurements();
    }
    if (!_measurementCovariance)
    {
        return false;
    }
    try
    {
        math::SquareMatrix positionCovariance(NdsCalculation::DIMENSION);
        for (std::size_t row = 0; row < NdsCalculation::DIMENSION; ++row)
        {
            for (std::size_t column = 0; column < NdsCalculation::DIMENSION; ++column)
            {
                positionCovariance(row, column) = (*state.errorCovariance())(row, column);
            }
        }
        _calculation = NdsCalculation::from(positionCovariance + *_measurementCovariance);
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
    std::optional<math::Length> maximumTravel
) const {
    return _calculation ? _calculation->calculate(observation, reference, maximumTravel) : std::nullopt;
}

} // namespace target
