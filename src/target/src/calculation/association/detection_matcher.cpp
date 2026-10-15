#include "detection_matcher.hpp"

#include "domain/target.hpp"
#include "validation/detection_validation.hpp"

#include <math/geometry/cartesian_position.hpp>
#include <math/geometry/spherical_direction.hpp>
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

using namespace math::literals;

const math::LinearVelocity MAX_ASSOCIATION_SPEED = 1000.0_mps;
const math::LinearVelocity MAX_INITIAL_DOPPLER_DIFFERENCE = 30.0_mps;
constexpr std::size_t NDS_DIMENSION = 3;

const math::Length MEASUREMENT_STANDARD_DEVIATION = 7.0_m;
// Measurement-only approximation: two independent measurements, each axis sigma=7m.
// Search gating should use predicted covariance once the tracking strategy provides it.
const math::SquareMatrix MEASUREMENT_DIFFERENCE_COVARIANCE =
    math::SquareMatrix::identity(NDS_DIMENSION)
    * (2.0 * MEASUREMENT_STANDARD_DEVIATION.m() * MEASUREMENT_STANDARD_DEVIATION.m());

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

bool positionsEqual(
    const Target& observation,
    const math::Length& range,
    const math::Angle& azimuth,
    const math::Angle& elevation)
{
    // Preserve the current exact antenna-position policy until NDS is specified.
    return observation.slantRange().equals(range, math::Length::NONE_TOLERANCE)
        && observation.azimuth().equals(azimuth, math::Angle::NONE_TOLERANCE)
        && observation.elevation().equals(elevation, math::Angle::NONE_TOLERANCE);
}

} // namespace

DetectionMatcher::DetectionMatcher(double positionNdsThreshold)
    : _positionNdsThreshold(positionNdsThreshold)
{
    if (!std::isfinite(_positionNdsThreshold) || _positionNdsThreshold < 0.0)
    {
        throw std::invalid_argument("Position NDS threshold must be finite and nonnegative");
    }
}

bool DetectionMatcher::isValidPosition(const Target& observation) const
{
    return validation::isValidTarget(observation);
}

std::optional<double> DetectionMatcher::matchesPosition(
    const Target& observation,
    const Target& other) const
{
    if (!isValidPosition(observation) || !isValidPosition(other))
    {
        return std::nullopt;
    }

    const bool tracking = other.state() == TargetState::TRACKING;
    const auto prediction = tracking
        ? other.predict(observation.lastObservationTime())
        : std::optional<TargetPrediction>{};
    if (tracking && !prediction)
    {
        return std::nullopt;
    }
    const auto referencePosition = prediction
        ? math::SphericalPosition(prediction->slantRange,
            math::SphericalDirection(prediction->azimuth, prediction->elevation)).toCartesian()
        : other.sphericalPosition().toCartesian();

    const auto position = observation.sphericalPosition().toCartesian();
    auto residual = math::Vector{position.x.m(), position.y.m(), position.z.m()}
        - math::Vector{referencePosition.x.m(), referencePosition.y.m(), referencePosition.z.m()};
    if (other.state() == TargetState::TENTATIVE)
    {
        const auto observationTime = observation.lastObservationTime();
        const auto initialTime = other.initialTime();
        const auto elapsed = observationTime >= initialTime
            ? observationTime - initialTime : initialTime - observationTime;
        const double seconds = std::chrono::duration<double>(elapsed).count();
        const double distanceMeters = residual.norm();
        const double maximumTravelMeters = MAX_ASSOCIATION_SPEED.mps() * seconds;
        if (!std::isfinite(distanceMeters) || !std::isfinite(maximumTravelMeters))
        {
            return std::nullopt;
        }

        // With the isotropic 98I covariance, gate only the distance beyond reachable motion.
        // This is equivalent to d <= maximumTravel + sqrt(98 * positionNdsThreshold).
        residual = distanceMeters <= maximumTravelMeters
            ? math::Vector(NDS_DIMENSION)
            : residual * ((distanceMeters - maximumTravelMeters) / distanceMeters);
    }
    const auto score = calculateNds(residual, MEASUREMENT_DIFFERENCE_COVARIANCE);
    if (!score || *score > _positionNdsThreshold)
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
        || !isValidPosition(observation) || !isValidPosition(tentative)
        || time <= tentative.lastResultTime() || time <= tentative.initialTime())
    {
        return std::nullopt;
    }

    const auto position = observation.sphericalPosition().toCartesian();
    const auto initialPosition = tentative.sphericalPosition().toCartesian();
    const auto displacement = math::Vector{position.x.m(), position.y.m(), position.z.m()}
        - math::Vector{initialPosition.x.m(), initialPosition.y.m(), initialPosition.z.m()};
    // Ordered nonnegative times keep this integer subtraction within its range.
    const double seconds = std::chrono::duration<double>(time - tentative.initialTime()).count();
    const double speedMagnitude = (displacement / seconds).norm();
    if (!std::isfinite(speedMagnitude))
    {
        return std::nullopt;
    }
    const auto speed = math::LinearVelocity::fromMetersPerSecond(speedMagnitude);
    if (speed.greaterThan(MAX_ASSOCIATION_SPEED, math::LinearVelocity::NONE_TOLERANCE)
        || !observation.dopplerVelocity().equals(
            tentative.dopplerVelocity(),
            math::QuantityTolerance<math::LinearVelocity>{MAX_INITIAL_DOPPLER_DIFFERENCE, 0.0}))
    {
        return std::nullopt;
    }

    return calculateNds(displacement, MEASUREMENT_DIFFERENCE_COVARIANCE);
}

bool DetectionMatcher::matchesTracking(
    const Target& observation,
    const Target& tracking) const
{
    const auto time = observation.lastObservationTime();
    if (tracking.state() != TargetState::TRACKING || !isValidPosition(observation)
        || time <= tracking.lastResultTime())
    {
        return false;
    }

    const auto prediction = tracking.predict(time);
    return prediction && positionsEqual(observation, prediction->slantRange,
        prediction->azimuth, prediction->elevation);
}

} // namespace target
