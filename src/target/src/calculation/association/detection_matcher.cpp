#include "detection_matcher.hpp"

#include "calculation/tracking/cartesian_math.hpp"
#include "domain/target.hpp"
#include "validation/detection_validation.hpp"

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

const math::Velocity MAX_INITIAL_SPEED = 1000.0_mps;
const math::Velocity MAX_INITIAL_DOPPLER_DIFFERENCE = 30.0_mps;
constexpr std::size_t NDS_DIMENSION = 3;

const math::Distance MEASUREMENT_STANDARD_DEVIATION = 7.0_m;
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
        || !residual.isFinite() || !covariance.isFinite()
        || !covariance.equals(covariance.transpose(),
            math::Tolerance{0.0, math::DEFAULT_RELATIVE_TOLERANCE}))
    {
        return std::nullopt;
    }

    // The math API has no SPD solve yet. Factor S = L L^T without forming S^-1.
    math::SquareMatrix lower(NDS_DIMENSION);
    for (std::size_t row = 0; row < NDS_DIMENSION; ++row)
    {
        for (std::size_t column = 0; column <= row; ++column)
        {
            const double first = covariance(row, column);
            const double second = covariance(column, row);
            // Symmetrize only differences already accepted by the numeric tolerance.
            double entry = first + 0.5 * (second - first);
            if (!std::isfinite(entry))
            {
                return std::nullopt;
            }

            for (std::size_t index = 0; index < column; ++index)
            {
                const double product = lower(row, index) * lower(column, index);
                entry -= product;
                if (!std::isfinite(product) || !std::isfinite(entry))
                {
                    return std::nullopt;
                }
            }

            if (row == column)
            {
                if (entry <= 0.0)
                {
                    return std::nullopt;
                }
                // Temporary std fallback until the math module provides an SPD solve.
                lower(row, column) = std::sqrt(entry);
            }
            else
            {
                // Direct division avoids the math Matrix scalar reciprocal overflow.
                lower(row, column) = entry / lower(column, column);
            }
            if (!std::isfinite(lower(row, column)))
            {
                return std::nullopt;
            }
        }
    }

    // Solve L y = r. Its squared norm is r^T S^-1 r.
    math::Vector transformed(NDS_DIMENSION);
    for (std::size_t row = 0; row < NDS_DIMENSION; ++row)
    {
        double value = residual[row];
        for (std::size_t column = 0; column < row; ++column)
        {
            const double product = lower(row, column) * transformed[column];
            value -= product;
            if (!std::isfinite(product) || !std::isfinite(value))
            {
                return std::nullopt;
            }
        }
        transformed[row] = value / lower(row, row);
        if (!std::isfinite(transformed[row]))
        {
            return std::nullopt;
        }
    }

    // norm() avoids unnecessary underflow in individual squared components.
    const double norm = transformed.norm();
    const double score = norm * norm;
    return std::isfinite(score) ? std::optional<double>{score} : std::nullopt;
}

bool positionsEqual(
    const Target& observation,
    const math::Distance& range,
    const math::Angle& azimuth,
    const math::Angle& elevation)
{
    // Preserve the current exact antenna-position policy until NDS is specified.
    return observation.slantRange().equals(range, math::Distance::NONE_TOLERANCE)
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

    auto referencePosition = cartesian::position(other);
    if (other.state() == TargetState::TRACKING)
    {
        const auto prediction = other.predict(observation.lastObservationTime());
        if (!prediction)
        {
            return std::nullopt;
        }
        referencePosition = cartesian::position(
            prediction->slantRange, prediction->azimuth, prediction->elevation);
    }

    const auto residual = cartesian::toVector(cartesian::position(observation))
        - cartesian::toVector(referencePosition);
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

    const auto displacement = cartesian::toVector(cartesian::position(observation))
        - cartesian::toVector(cartesian::position(tentative));
    // Ordered nonnegative times keep this integer subtraction within its range.
    const double seconds = std::chrono::duration<double>(time - tentative.initialTime()).count();
    const auto speed = math::Velocity::fromMetersPerSecond((displacement / seconds).norm());
    if (!speed.isFinite()
        || speed.greaterThan(MAX_INITIAL_SPEED, math::Velocity::NONE_TOLERANCE)
        || !observation.dopplerVelocity().equals(
            tentative.dopplerVelocity(),
            math::QuantityTolerance<math::Velocity>{MAX_INITIAL_DOPPLER_DIFFERENCE, 0.0}))
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
