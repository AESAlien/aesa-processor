#include "detection_matcher.hpp"

#include "domain/target.hpp"

#include <math/geometry/cartesian_displacement.hpp>
#include <math/geometry/cartesian_position.hpp>
#include <math/geometry/spherical_position.hpp>
#include <math/linear_algebra/square_matrix.hpp>
#include <math/linear_algebra/vector.hpp>
#include <math/quantity/quantity_tolerance.hpp>
#include <math/tolerance.hpp>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>

namespace target
{
namespace
{

constexpr std::size_t NDS_DIMENSION = 3;

math::SquareMatrix measurementCovariance(const TargetConfig& config)
{
    const double sigma = config.measurementStandardDeviation.m();
    return math::SquareMatrix::identity(NDS_DIMENSION) * (sigma * sigma);
}

math::SquareMatrix measurementDifferenceCovariance(const TargetConfig& config)
{
    const double sigma = config.measurementStandardDeviation.m();
    return math::SquareMatrix::identity(NDS_DIMENSION) * (2.0 * sigma * sigma);
}

math::SquareMatrix innovationCovariance(
    const std::optional<math::SquareMatrix>& positionCovariance,
    const math::SquareMatrix& measurement,
    const math::SquareMatrix& legacyDifference)
{
    // CV and existing strategy implementations may not estimate covariance.
    // Their explicit legacy fallback is two independent measurement errors, 2R.
    return positionCovariance ? *positionCovariance + measurement : legacyDifference;
}

// Cartesian residual components are metres; covariance entries are square metres.
// Returns the dimensionless squared distance r^T S^-1 r, not a gated decision.
std::optional<double> calculateNds(
    const math::Vector& residual,
    const math::SquareMatrix& covariance)
{
    if (residual.size() != NDS_DIMENSION || covariance.size() != NDS_DIMENSION
        || !residual.isFinite()
        || !covariance.isSymmetric(math::Tolerance{0.0, math::DEFAULT_RELATIVE_TOLERANCE}))
    {
        return std::nullopt;
    }

    // Preserve the accepted roundoff tolerance before math's exact symmetry check.
    auto symmetricCovariance = covariance;
    for (std::size_t row = 0; row < NDS_DIMENSION; ++row)
    {
        for (std::size_t column = 0; column < row; ++column)
        {
            const double first = covariance(row, column);
            const double second = covariance(column, row);
            const double entry = first + 0.5 * (second - first);
            if (!std::isfinite(entry))
            {
                return std::nullopt;
            }
            symmetricCovariance(row, column) = entry;
            symmetricCovariance(column, row) = entry;
        }
    }

    try
    {
        const auto lower = math::cholesky(symmetricCovariance);
        const auto transformed = lower.solveLowerTriangular(residual);

        // Solve L y = r: ||y||^2 = r^T S^-1 r. norm() preserves tiny components.
        const double norm = transformed.norm();
        const double score = norm * norm;
        return std::isfinite(score) ? std::optional<double>{score} : std::nullopt;
    }
    catch (const std::domain_error&)
    {
        return std::nullopt;
    }
}

} // namespace

DetectionMatcher::DetectionMatcher(TargetConfig config)
    : _config(config)
    , _measurementCovariance(measurementCovariance(_config))
    , _measurementDifferenceCovariance(measurementDifferenceCovariance(_config))
{
}

std::optional<double> DetectionMatcher::matchesPosition(
    const Target& observation,
    const Target& other) const
{
    const auto observationTime = observation.lastObservationTime();
    const auto referenceTime = other.lastObservationTime();
    const bool tracking = other.state() == TargetState::TRACKING;
    const bool pastTracking = tracking && observationTime < referenceTime;
    const bool predictTracking = tracking && !pastTracking;
    const auto prediction = predictTracking
        ? other.predict(observationTime)
        : std::optional<TargetPrediction>{};
    if (predictTracking && !prediction)
    {
        return std::nullopt;
    }
    const auto referencePosition = prediction
        ? prediction->sphericalPosition.toCartesian()
        : other.sphericalPosition().toCartesian();

    const auto position = observation.sphericalPosition().toCartesian();
    math::Vector residual(NDS_DIMENSION);
    try
    {
        residual = (position - referencePosition).toVector();
    }
    catch (const std::invalid_argument&)
    {
        // Valid positions can still have an unrepresentable displacement.
        return std::nullopt;
    }
    if (other.state() == TargetState::TENTATIVE || pastTracking)
    {
        // Older search results use the latest stored position without rewinding the filter.
        const auto elapsed = observationTime >= referenceTime
            ? observationTime - referenceTime : referenceTime - observationTime;
        const double seconds = std::chrono::duration<double>(elapsed).count();
        const double distanceMeters = residual.norm();
        const double maximumTravelMeters = _config.maxAssociationSpeed.mps() * seconds;
        if (!std::isfinite(distanceMeters) || !std::isfinite(maximumTravelMeters))
        {
            return std::nullopt;
        }

        // Retain the geometric motion allowance by shrinking the residual radially
        // before scoring it with the available position uncertainty.
        residual = distanceMeters <= maximumTravelMeters
            ? math::Vector(NDS_DIMENSION)
            : residual * ((distanceMeters - maximumTravelMeters) / distanceMeters);
    }
    auto covariance = _measurementDifferenceCovariance;
    if (prediction)
    {
        covariance = innovationCovariance(prediction->positionCovariance,
            _measurementCovariance, _measurementDifferenceCovariance);
    }
    else if (pastTracking && other.trackingState()->errorCovariance)
    {
        // Delayed searches compare against the latest stored position without
        // rewinding the filter, while retaining that position's uncertainty.
        math::SquareMatrix positionCovariance(NDS_DIMENSION);
        const auto& stateCovariance = *other.trackingState()->errorCovariance;
        for (std::size_t row = 0; row < NDS_DIMENSION; ++row)
        {
            for (std::size_t column = 0; column < NDS_DIMENSION; ++column)
            {
                positionCovariance(row, column) = stateCovariance(row, column);
            }
        }
        covariance = positionCovariance + _measurementCovariance;
    }
    const auto score = calculateNds(residual, covariance);
    if (!score || *score > _config.positionNdsThreshold)
    {
        return std::nullopt;
    }
    return score;
}

std::optional<double> DetectionMatcher::matchesConfirmation(
    const Target& observation,
    const Target& tentative) const
{
    const auto time = observation.lastObservationTime();
    if (tentative.state() != TargetState::TENTATIVE
        || time <= tentative.lastResultTime() || time <= tentative.initialTime())
    {
        return std::nullopt;
    }

    const auto position = observation.sphericalPosition().toCartesian();
    const auto initialPosition = tentative.sphericalPosition().toCartesian();
    try
    {
        const auto displacement = position - initialPosition;
        // Ordered nonnegative times keep this integer subtraction within its range.
        const auto elapsed = std::chrono::duration<double>(time - tentative.initialTime());
        const auto speed = displacement.toVelocity(elapsed).speed();
        if (speed.greaterThan(_config.maxAssociationSpeed, math::LinearVelocity::NONE_TOLERANCE)
            || !observation.dopplerVelocity().equals(
                tentative.dopplerVelocity(),
                math::QuantityTolerance<math::LinearVelocity>{_config.maxInitialDopplerDifference, 0.0}))
        {
            return std::nullopt;
        }

        return calculateNds(displacement.toVector(), _measurementDifferenceCovariance);
    }
    catch (const std::invalid_argument&)
    {
        return std::nullopt;
    }
}

std::optional<double> DetectionMatcher::matchesTracking(
    const Target& observation,
    const Target& tracking) const
{
    const auto time = observation.lastObservationTime();
    if (tracking.state() != TargetState::TRACKING
        || time <= tracking.lastResultTime())
    {
        return std::nullopt;
    }

    const auto prediction = tracking.predict(time);
    if (!prediction)
    {
        return std::nullopt;
    }

    const auto position = observation.sphericalPosition().toCartesian();
    const auto predictedPosition = prediction->sphericalPosition.toCartesian();
    try
    {
        const auto residual = (position - predictedPosition).toVector();
        const auto covariance = innovationCovariance(prediction->positionCovariance,
            _measurementCovariance, _measurementDifferenceCovariance);
        const auto score = calculateNds(residual, covariance);
        if (!score || *score > _config.positionNdsThreshold)
        {
            return std::nullopt;
        }
        return score;
    }
    catch (const std::invalid_argument&)
    {
        // Valid positions can still have an unrepresentable displacement.
        return std::nullopt;
    }
}

} // namespace target
