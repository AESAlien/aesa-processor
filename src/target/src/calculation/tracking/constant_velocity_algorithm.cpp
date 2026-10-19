#include "constant_velocity_algorithm.hpp"

#include "domain/target.hpp"

#include <math/geometry/cartesian_displacement.hpp>
#include <math/geometry/cartesian_position.hpp>
#include <math/geometry/cartesian_velocity.hpp>
#include <math/geometry/spherical_position.hpp>
#include <math/linear_algebra/vector.hpp>

#include <cmath>

namespace target
{
namespace
{

using namespace math::literals;

double elapsedSeconds(
    std::chrono::milliseconds previous,
    std::chrono::milliseconds current)
{
    // Validated nonnegative, ordered times keep the integer difference in range.
    // Subtract before conversion to preserve short intervals at large timestamps.
    return std::chrono::duration<double>(current - previous).count();
}

std::optional<TargetTrackingState> estimateTrackingState(
    const math::CartesianPosition& previousPosition,
    std::chrono::milliseconds previousTime,
    const Target& observation)
{
    const auto time = observation.lastObservationTime();
    const auto position = observation.sphericalPosition().toCartesian();
    const auto velocity = (position - previousPosition).toVelocity(
        std::chrono::duration<double>{elapsedSeconds(previousTime, time)});
    // The temporary CV strategy leaves acceleration zero and covariance unestimated.
    return TargetTrackingState{observation.sphericalPosition(),
        observation.dopplerVelocity(), observation.power_db(), time, position, velocity};
}

} // namespace

std::optional<TargetTrackingState> ConstantVelocityAlgorithm::initialize(
    const Target& first,
    const Target& second) const
{
    const auto firstTime = first.lastObservationTime();
    const auto secondTime = second.lastObservationTime();
    if (secondTime <= firstTime)
    {
        return std::nullopt;
    }

    return estimateTrackingState(first.sphericalPosition().toCartesian(), firstTime, second);
}

std::optional<TargetTrackingState> ConstantVelocityAlgorithm::update(
    const Target& current,
    const Target& observation) const
{
    const auto& currentState = current.trackingState();
    const auto time = observation.lastObservationTime();
    if (!currentState || time <= currentState->timestamp)
    {
        return std::nullopt;
    }

    return estimateTrackingState(currentState->position, currentState->timestamp, observation);
}

std::optional<TargetPrediction> ConstantVelocityAlgorithm::predict(
    const Target& current,
    std::chrono::milliseconds time) const
{
    const auto& currentState = current.trackingState();
    if (!currentState || time < currentState->timestamp)
    {
        return std::nullopt;
    }

    const auto& state = *currentState;
    // Preserve a stationary measured position without conversion roundoff.
    const auto& velocity = state.velocity;
    if (velocity.x.equals(0.0_mps, math::LinearVelocity::NONE_TOLERANCE)
        && velocity.y.equals(0.0_mps, math::LinearVelocity::NONE_TOLERANCE)
        && velocity.z.equals(0.0_mps, math::LinearVelocity::NONE_TOLERANCE))
    {
        return TargetPrediction{time, state.sphericalPosition};
    }

    const math::Vector velocityVector{velocity.x.mps(), velocity.y.mps(), velocity.z.mps()};
    const auto positionVector = math::Vector{
        state.position.x.m(), state.position.y.m(), state.position.z.m()}
        + velocityVector * elapsedSeconds(state.timestamp, time);
    const double rangeMeters = positionVector.norm();
    if (!positionVector.isFinite() || !std::isfinite(rangeMeters) || rangeMeters <= 0.0)
    {
        return std::nullopt;
    }

    const math::CartesianPosition predictedPosition{
        math::Length::fromMeters(positionVector[0]),
        math::Length::fromMeters(positionVector[1]),
        math::Length::fromMeters(positionVector[2])};
    const auto position = predictedPosition.toSpherical();
    return TargetPrediction{time, position};
}

} // namespace target
