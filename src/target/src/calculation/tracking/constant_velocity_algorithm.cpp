#include "constant_velocity_algorithm.hpp"

#include "cartesian_math.hpp"
#include "validation/detection_validation.hpp"

#include <array>
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
    const std::array<math::Distance, 3>& previousPosition,
    std::chrono::milliseconds previousTime,
    const Target& observation)
{
    const auto time = observation.lastObservationTime();
    const auto position = cartesian::position(observation);
    const auto previousPositionVector = cartesian::toVector(previousPosition);
    const auto positionVector = cartesian::toVector(position);
    if (!previousPositionVector.isFinite() || !positionVector.isFinite())
    {
        return std::nullopt;
    }

    const auto velocityVector = (positionVector - previousPositionVector)
        / elapsedSeconds(previousTime, time);
    if (!velocityVector.isFinite() || !std::isfinite(velocityVector.norm()))
    {
        return std::nullopt;
    }

    return TargetTrackingState{observation.slantRange(), observation.azimuth(), observation.elevation(),
        observation.dopplerVelocity(), observation.power_db(), time, position,
        cartesian::toVelocity(velocityVector)};
}

} // namespace

std::optional<TargetTrackingState> ConstantVelocityAlgorithm::initialize(
    const Target& first,
    const Target& second) const
{
    const auto firstTime = first.lastObservationTime();
    const auto secondTime = second.lastObservationTime();
    if (!validation::isValidTarget(first) || !validation::isValidTarget(second)
        || secondTime <= firstTime)
    {
        return std::nullopt;
    }

    return estimateTrackingState(cartesian::position(first), firstTime, second);
}

std::optional<TargetTrackingState> ConstantVelocityAlgorithm::update(
    const Target& current,
    const Target& observation) const
{
    const auto& currentState = current.trackingState();
    const auto time = observation.lastObservationTime();
    if (!currentState || !validation::isValidTrackingState(*currentState)
        || !validation::isValidTarget(observation) || time <= currentState->timestamp)
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
    if (!currentState || !validation::isValidTrackingState(*currentState)
        || time < currentState->timestamp)
    {
        return std::nullopt;
    }

    const auto& state = *currentState;
    // Preserve a stationary measured position exactly while matching remains exact.
    const auto& velocity = state.velocity;
    if (velocity[0].equals(0.0_mps, math::Velocity::NONE_TOLERANCE)
        && velocity[1].equals(0.0_mps, math::Velocity::NONE_TOLERANCE)
        && velocity[2].equals(0.0_mps, math::Velocity::NONE_TOLERANCE))
    {
        return TargetPrediction{time, state.slantRange, state.azimuth, state.elevation};
    }

    const auto positionVector = cartesian::toVector(state.position)
        + cartesian::toVector(velocity) * elapsedSeconds(state.timestamp, time);
    const auto range = math::Distance::fromMeters(positionVector.norm());
    if (!positionVector.isFinite() || !range.isFinite()
        || range.lessThanOrEqual(0.0_m, math::Distance::NONE_TOLERANCE))
    {
        return std::nullopt;
    }

    const auto position = cartesian::toPosition(positionVector);
    const auto horizontalRange = math::Distance::fromMeters(
        math::Vector{position[0].m(), position[1].m()}.norm());
    return TargetPrediction{
        time,
        range,
        math::Angle::fromRadians(std::atan2(position[1].m(), position[0].m())),
        math::Angle::fromRadians(std::atan2(position[2].m(), horizontalRange.m()))
    };
}

} // namespace target
