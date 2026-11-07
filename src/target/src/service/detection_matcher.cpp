#include "detection_matcher.hpp"

#include "domain/tentative_track.hpp"

#include <math/geometry/cartesian_displacement.hpp>
#include <math/geometry/cartesian_position.hpp>
#include <math/geometry/spherical_position.hpp>
#include <math/linear_algebra/square_matrix.hpp>
#include <math/linear_algebra/vector.hpp>
#include <math/quantity/quantity_tolerance.hpp>

#include <chrono>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <utility>

namespace target
{
namespace
{

constexpr std::size_t POSITION_DIMENSION = NdsAlgorithm::DIMENSION;

math::SquareMatrix measurementCovariance(const MeasurementConfig& config)
{
    const double sigma = config.standardDeviation().m();
    return math::SquareMatrix::identity(POSITION_DIMENSION) * (sigma * sigma);
}

math::SquareMatrix measurementDifferenceCovariance(const MeasurementConfig& config)
{
    const double sigma = config.standardDeviation().m();
    return math::SquareMatrix::identity(POSITION_DIMENSION) * (2.0 * sigma * sigma);
}

math::SquareMatrix innovationCovariance(
    const std::optional<math::SquareMatrix>& positionCovariance,
    const math::SquareMatrix& measurement,
    const math::SquareMatrix& legacyDifference
) {
    // CV and existing strategy implementations may not estimate covariance.
    // Their explicit legacy fallback is two independent measurement errors, 2R.
    return positionCovariance ? *positionCovariance + measurement : legacyDifference;
}

} // namespace

TrackingAssociation::TrackingAssociation(
    std::chrono::milliseconds time,
    math::CartesianPosition position,
    NdsAlgorithm ndsAlgorithm,
    double ndsThreshold
) : _time(time),
    _position(std::move(position)),
    _ndsAlgorithm(std::move(ndsAlgorithm)),
    _ndsThreshold(ndsThreshold)
{
}

std::optional<double> TrackingAssociation::matches(const Observation& observation) const
{
    if (observation.lastObservationTime() != _time)
    {
        return std::nullopt;
    }

    const auto position = observation.sphericalPosition().toCartesian();
    try
    {
        const auto residual = (position - _position).toVector();
        const auto score = _ndsAlgorithm.calculate(residual);
        return score && *score <= _ndsThreshold ? score : std::nullopt;
    }
    catch (const std::invalid_argument&)
    {
        // Valid positions can still have an unrepresentable displacement.
        return std::nullopt;
    }
}

DetectionMatcher::DetectionMatcher(
    AssociationConfig config,
    const MeasurementConfig& measurementConfig
) : _config(std::move(config)),
    _measurementCovariance(measurementCovariance(measurementConfig)),
    _measurementDifferenceCovariance(measurementDifferenceCovariance(measurementConfig))
{
}

std::optional<double> DetectionMatcher::positionScore(
    const Observation& observation,
    const math::SphericalPosition& referencePosition,
    std::chrono::milliseconds referenceTime,
    const math::SquareMatrix& covariance,
    bool compensateMotion
) const {
    try
    {
        const auto displacement = observation.sphericalPosition().toCartesian()
            - referencePosition.toCartesian();
        auto residual = displacement.toVector();
        if (compensateMotion)
        {
            const auto elapsed = std::chrono::abs(observation.lastObservationTime() - referenceTime);
            const auto distance = displacement.distance();
            const auto maximumTravel = _config.maxAssociationSpeed() * elapsed;
            residual = distance <= maximumTravel
                ? math::Vector(POSITION_DIMENSION)
                : residual * ((distance - maximumTravel) / distance);
        }
        const auto score = NdsAlgorithm::calculate(residual, covariance);
        return score && *score <= _config.positionNdsThreshold() ? score : std::nullopt;
    }
    catch (const std::invalid_argument&)
    {
        return std::nullopt;
    }
}

std::optional<double> DetectionMatcher::matchesPosition(
    const Observation& observation,
    const TentativeTrack& other
) const {
    const auto& first = other.observation();
    return positionScore(
        observation,
        first.sphericalPosition(),
        first.lastObservationTime(),
        _measurementDifferenceCovariance,
        true
    );
}

std::optional<double> DetectionMatcher::matchesPosition(
    const Observation& observation,
    const ConfirmedTrack& other
) const {
    const auto time = observation.lastObservationTime();
    const auto referenceTime = other.lastObservationTime();
    if (time >= referenceTime)
    {
        const auto prediction = other.predict(time);
        if (!prediction)
        {
            return std::nullopt;
        }
        const auto covariance = innovationCovariance(
            prediction->positionCovariance(),
            _measurementCovariance,
            _measurementDifferenceCovariance
        );
        return positionScore(
            observation,
            prediction->sphericalPosition(),
            referenceTime,
            covariance,
            false
        );
    }

    auto covariance = _measurementDifferenceCovariance;
    if (other.state()->errorCovariance())
    {
        math::SquareMatrix positionCovariance(POSITION_DIMENSION);
        const auto& stateCovariance = *other.state()->errorCovariance();
        for (std::size_t row = 0; row < POSITION_DIMENSION; ++row)
        {
            for (std::size_t column = 0; column < POSITION_DIMENSION; ++column)
            {
                positionCovariance(row, column) = stateCovariance(row, column);
            }
        }
        covariance = positionCovariance + _measurementCovariance;
    }
    return positionScore(observation, other.sphericalPosition(), referenceTime, covariance, true);
}

std::optional<double> DetectionMatcher::matchesConfirmation(
    const Observation& observation,
    const TentativeTrack& tentative
) const {
    const auto time = observation.lastObservationTime();
    if (time <= tentative.lastResultTime())
    {
        return std::nullopt;
    }

    const auto position = observation.sphericalPosition().toCartesian();
    const auto initialPosition = tentative.observation().sphericalPosition().toCartesian();
    try
    {
        const auto displacement = position - initialPosition;
        // Ordered nonnegative times keep this integer subtraction within its range.
        const auto elapsed = std::chrono::duration<double>(time - tentative.lastResultTime());
        const auto speed = displacement.toVelocity(elapsed).speed();
        if (speed.greaterThan(_config.maxAssociationSpeed(), math::LinearVelocity::NONE_TOLERANCE) ||
            !observation.dopplerVelocity().equals(
                tentative.observation().dopplerVelocity(),
                math::QuantityTolerance<math::LinearVelocity>(_config.maxInitialDopplerDifference(), 0.0)
            ))
        {
            return std::nullopt;
        }

        return NdsAlgorithm::calculate(displacement.toVector(), _measurementDifferenceCovariance);
    }
    catch (const std::invalid_argument&)
    {
        return std::nullopt;
    }
}

std::optional<double> DetectionMatcher::matchesTracking(
    const Observation& observation,
    const ConfirmedTrack& tracking
) const {
    const auto association = prepareTracking(tracking, observation.lastObservationTime());
    return association ? association->matches(observation) : std::nullopt;
}

std::optional<TrackingAssociation> DetectionMatcher::prepareTracking(
    const ConfirmedTrack& tracking,
    std::chrono::milliseconds time
) const {
    if (time <= tracking.lastResultTime())
    {
        return std::nullopt;
    }

    const auto prediction = tracking.predict(time);
    if (!prediction)
    {
        return std::nullopt;
    }

    const auto predictedPosition = prediction->sphericalPosition().toCartesian();
    try
    {
        const auto covariance = innovationCovariance(
            prediction->positionCovariance(),
            _measurementCovariance,
            _measurementDifferenceCovariance
        );
        auto ndsAlgorithm = NdsAlgorithm::from(covariance);
        if (!ndsAlgorithm)
        {
            return std::nullopt;
        }
        return TrackingAssociation(
            time,
            predictedPosition,
            std::move(*ndsAlgorithm),
            _config.positionNdsThreshold()
        );
    }
    catch (const std::invalid_argument&)
    {
        return std::nullopt;
    }
}

} // namespace target
