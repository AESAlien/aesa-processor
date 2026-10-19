#include "constant_acceleration_algorithm.hpp"

#include "domain/target.hpp"

#include <math/geometry/cartesian_displacement.hpp>
#include <math/geometry/cartesian_position.hpp>
#include <math/geometry/cartesian_velocity.hpp>
#include <math/geometry/spherical_position.hpp>
#include <math/linear_algebra/matrix.hpp>

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>

namespace target
{
namespace
{

constexpr std::size_t POSITION_DIMENSION = 3;
constexpr std::size_t VELOCITY_OFFSET = 3;
constexpr std::size_t ACCELERATION_OFFSET = 6;
constexpr std::size_t STATE_DIMENSION = TargetTrackingState::STATE_DIMENSION;

double elapsedSeconds(
    std::chrono::milliseconds previous,
    std::chrono::milliseconds current)
{
    return std::chrono::duration<double>(current - previous).count();
}

math::Vector stateVector(const TargetTrackingState& state)
{
    return math::Vector{state.position.x.m(), state.position.y.m(), state.position.z.m(),
        state.velocity.x.mps(), state.velocity.y.mps(), state.velocity.z.mps(),
        state.acceleration[0], state.acceleration[1], state.acceleration[2]};
}

math::Matrix measurementMatrix()
{
    math::Matrix measurement(POSITION_DIMENSION, STATE_DIMENSION);
    for (std::size_t axis = 0; axis < POSITION_DIMENSION; ++axis)
    {
        measurement(axis, axis) = 1.0;
    }
    return measurement;
}

std::optional<math::CartesianVelocity> initialVelocity(
    const math::CartesianPosition& first,
    const math::CartesianPosition& second,
    double seconds)
{
    try
    {
        const auto velocity = (second - first).toVelocity(std::chrono::duration<double>(seconds));
        const math::Vector components{velocity.x.mps(), velocity.y.mps(), velocity.z.mps()};
        return std::isfinite(components.norm())
            ? std::optional<math::CartesianVelocity>(velocity) : std::nullopt;
    }
    catch (const std::invalid_argument&)
    {
        // Finite positions can still produce an unrepresentable difference or speed.
        return std::nullopt;
    }
}

math::SquareMatrix positionCovariance(const math::SquareMatrix& covariance)
{
    math::SquareMatrix position(POSITION_DIMENSION);
    for (std::size_t row = 0; row < POSITION_DIMENSION; ++row)
    {
        for (std::size_t column = 0; column < POSITION_DIMENSION; ++column)
        {
            position(row, column) = covariance(row, column);
        }
    }
    return position;
}

std::optional<math::SquareMatrix> symmetricCovariance(math::SquareMatrix covariance)
{
    if (!covariance.isFinite())
    {
        return std::nullopt;
    }
    for (std::size_t row = 0; row < covariance.size(); ++row)
    {
        if (covariance(row, row) < 0.0)
        {
            return std::nullopt;
        }
        for (std::size_t column = 0; column < row; ++column)
        {
            // Repair only arithmetic roundoff; both entries receive the same value.
            const double entry = 0.5 * covariance(row, column)
                + 0.5 * covariance(column, row);
            covariance(row, column) = entry;
            covariance(column, row) = entry;
        }
    }
    return covariance;
}

struct PredictedState
{
    math::Vector values;
    math::SquareMatrix covariance;
};

std::optional<PredictedState> predictState(
    const TargetTrackingState& state,
    std::chrono::milliseconds time)
{
    if (!state.errorCovariance || time < state.timestamp)
    {
        return std::nullopt;
    }

    const double seconds = elapsedSeconds(state.timestamp, time);
    const double halfSquaredSeconds = 0.5 * seconds * seconds;
    if (!std::isfinite(halfSquaredSeconds))
    {
        return std::nullopt;
    }
    auto transition = math::SquareMatrix::identity(STATE_DIMENSION);
    for (std::size_t axis = 0; axis < POSITION_DIMENSION; ++axis)
    {
        transition(axis, axis + VELOCITY_OFFSET) = seconds;
        transition(axis, axis + ACCELERATION_OFFSET) = halfSquaredSeconds;
        transition(axis + VELOCITY_OFFSET, axis + ACCELERATION_OFFSET) = seconds;
    }
    auto values = transition * stateVector(state);
    auto covariance = symmetricCovariance(
        transition * *state.errorCovariance * math::transpose(transition));
    if (!values.isFinite() || !covariance)
    {
        return std::nullopt;
    }
    // Q is zero: no process-noise covariance is added during prediction.
    return PredictedState{std::move(values), std::move(*covariance)};
}

std::optional<math::CartesianPosition> calculatedPosition(const math::Vector& values)
{
    const math::Vector position{values[0], values[1], values[2]};
    const double range = position.norm();
    if (!position.isFinite() || !std::isfinite(range) || range <= 0.0)
    {
        return std::nullopt;
    }
    return math::CartesianPosition(math::Length::fromMeters(values[0]),
        math::Length::fromMeters(values[1]), math::Length::fromMeters(values[2]));
}

} // namespace

ConstantAccelerationAlgorithm::ConstantAccelerationAlgorithm(TargetConfig config)
    : _config(std::move(config))
{
}

std::optional<TargetTrackingState> ConstantAccelerationAlgorithm::initialize(
    const Target& first,
    const Target& second) const
{
    const auto firstTime = first.lastObservationTime();
    const auto secondTime = second.lastObservationTime();
    if (secondTime <= firstTime)
    {
        return std::nullopt;
    }

    const double seconds = elapsedSeconds(firstTime, secondTime);
    const auto firstPosition = first.sphericalPosition().toCartesian();
    const auto position = second.sphericalPosition().toCartesian();
    const auto velocity = initialVelocity(firstPosition, position, seconds);
    if (!velocity)
    {
        return std::nullopt;
    }
    const double sigma = _config.measurementStandardDeviation.m();
    const double measurementVariance = sigma * sigma;
    const double accelerationVariance = _config.initialAccelerationStandardDeviation
        * _config.initialAccelerationStandardDeviation;
    const double positionVelocityCovariance = measurementVariance / seconds;
    const double velocityVariance = 2.0 * measurementVariance / seconds / seconds
        + 0.25 * seconds * seconds * accelerationVariance;
    const double velocityAccelerationCovariance = 0.5 * seconds * accelerationVariance;

    math::SquareMatrix initialCovariance(STATE_DIMENSION);
    for (std::size_t axis = 0; axis < POSITION_DIMENSION; ++axis)
    {
        initialCovariance(axis, axis) = measurementVariance;
        initialCovariance(axis, axis + VELOCITY_OFFSET) = positionVelocityCovariance;
        initialCovariance(axis + VELOCITY_OFFSET, axis) = positionVelocityCovariance;
        initialCovariance(axis + VELOCITY_OFFSET, axis + VELOCITY_OFFSET) = velocityVariance;
        initialCovariance(axis + VELOCITY_OFFSET, axis + ACCELERATION_OFFSET) = velocityAccelerationCovariance;
        initialCovariance(axis + ACCELERATION_OFFSET, axis + VELOCITY_OFFSET) = velocityAccelerationCovariance;
        initialCovariance(axis + ACCELERATION_OFFSET, axis + ACCELERATION_OFFSET) = accelerationVariance;
    }
    if (!initialCovariance.isFinite())
    {
        return std::nullopt;
    }
    return TargetTrackingState(position.toSpherical(), second.dopplerVelocity(),
        second.power_db(), secondTime, position, *velocity,
        math::Vector(POSITION_DIMENSION), std::move(initialCovariance));
}

std::optional<TargetTrackingState> ConstantAccelerationAlgorithm::update(
    const Target& current,
    const Target& observation) const
{
    const auto& state = current.trackingState();
    const auto time = observation.lastObservationTime();
    if (!state || time <= state->timestamp || time <= current.lastResultTime())
    {
        return std::nullopt;
    }
    const auto predicted = predictState(*state, time);
    if (!predicted)
    {
        return std::nullopt;
    }

    const auto observationPosition = observation.sphericalPosition().toCartesian();
    const math::Vector measurement{observationPosition.x.m(),
        observationPosition.y.m(), observationPosition.z.m()};
    const auto measurementMapping = measurementMatrix();
    const double sigma = _config.measurementStandardDeviation.m();
    const auto measurementNoise = math::SquareMatrix::identity(POSITION_DIMENSION) * (sigma * sigma);
    const auto crossCovariance = predicted->covariance.asMatrix() * math::transpose(measurementMapping);
    auto innovationCovariance = symmetricCovariance(
        positionCovariance(predicted->covariance) + measurementNoise);
    if (!innovationCovariance)
    {
        return std::nullopt;
    }

    // S = L L^T. Solve each transposed gain row instead of forming S^-1.
    // A non-positive-definite S is a math/domain error and deliberately propagates.
    const auto lower = math::cholesky(*innovationCovariance);
    const auto upper = math::transpose(lower);
    math::Matrix gain(STATE_DIMENSION, POSITION_DIMENSION);
    for (std::size_t row = 0; row < STATE_DIMENSION; ++row)
    {
        const math::Vector rightHandSide{crossCovariance(row, 0),
            crossCovariance(row, 1), crossCovariance(row, 2)};
        const auto solved = upper.solveUpperTriangular(lower.solveLowerTriangular(rightHandSide));
        for (std::size_t column = 0; column < POSITION_DIMENSION; ++column)
        {
            gain(row, column) = solved[column];
        }
    }
    const auto residual = measurement - measurementMapping * predicted->values;
    const auto values = predicted->values + gain * residual;
    // Joseph form retains the two PSD contributions despite rounding in K.
    const auto remaining = math::SquareMatrix::identity(STATE_DIMENSION).asMatrix()
        - gain * measurementMapping;
    auto covariance = symmetricCovariance(math::SquareMatrix(
        remaining * predicted->covariance.asMatrix() * math::transpose(remaining)
        + gain * measurementNoise.asMatrix() * math::transpose(gain)));
    if (!values.isFinite() || !covariance)
    {
        return std::nullopt;
    }
    const auto position = calculatedPosition(values);
    const math::Vector velocityValues{values[3], values[4], values[5]};
    if (!position || !std::isfinite(velocityValues.norm()))
    {
        return std::nullopt;
    }
    const math::CartesianVelocity velocity(math::LinearVelocity::fromMetersPerSecond(values[3]),
        math::LinearVelocity::fromMetersPerSecond(values[4]),
        math::LinearVelocity::fromMetersPerSecond(values[5]));
    return TargetTrackingState(position->toSpherical(), observation.dopplerVelocity(),
        observation.power_db(), time, *position, velocity,
        math::Vector{values[6], values[7], values[8]}, std::move(*covariance));
}

std::optional<TargetPrediction> ConstantAccelerationAlgorithm::predict(
    const Target& current,
    std::chrono::milliseconds time) const
{
    const auto& state = current.trackingState();
    if (!state)
    {
        return std::nullopt;
    }
    const auto predicted = predictState(*state, time);
    if (!predicted)
    {
        return std::nullopt;
    }
    const auto position = calculatedPosition(predicted->values);
    if (!position)
    {
        return std::nullopt;
    }
    return TargetPrediction(time, position->toSpherical(), positionCovariance(predicted->covariance));
}

} // namespace target
