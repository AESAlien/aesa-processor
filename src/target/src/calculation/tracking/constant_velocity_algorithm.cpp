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

bool isFinite(const math::CartesianPoint& position)
{
    return std::isfinite(position.x().m()) && std::isfinite(position.y().m())
        && std::isfinite(position.z().m());
}

bool isFinite(const std::array<math::Velocity, 3>& velocity)
{
    return std::isfinite(velocity[0].mps()) && std::isfinite(velocity[1].mps())
        && std::isfinite(velocity[2].mps());
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

    const auto firstPosition = cartesian::position(first);
    const auto secondPosition = cartesian::position(second);
    const auto velocityVector = (cartesian::toVector(secondPosition)
        - cartesian::toVector(firstPosition)) / elapsedSeconds(firstTime, secondTime);
    const auto velocity = cartesian::toVelocity(velocityVector);

    if (!isFinite(firstPosition) || !isFinite(secondPosition) || !isFinite(velocity)
        || !std::isfinite(velocityVector.norm()))
    {
        return std::nullopt;
    }

    return TargetTrackingState{second.slantRange(), second.azimuth(), second.elevation(),
        second.dopplerVelocity(), second.power_db(), secondTime, secondPosition, velocity};
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

    const auto& state = *currentState;
    const auto position = cartesian::position(observation);
    const auto velocityVector = (cartesian::toVector(position)
        - cartesian::toVector(state.position)) / elapsedSeconds(state.timestamp, time);
    const auto velocity = cartesian::toVelocity(velocityVector);

    if (!isFinite(position) || !isFinite(velocity) || !std::isfinite(velocityVector.norm()))
    {
        return std::nullopt;
    }

    return TargetTrackingState{observation.slantRange(), observation.azimuth(), observation.elevation(),
        observation.dopplerVelocity(), observation.power_db(), time, position, velocity};
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
    if (velocity[0].equals(0.0_mps, math::QuantityTolerance{0.0_mps, 0.0})
        && velocity[1].equals(0.0_mps, math::QuantityTolerance{0.0_mps, 0.0})
        && velocity[2].equals(0.0_mps, math::QuantityTolerance{0.0_mps, 0.0}))
    {
        return TargetPrediction{time, state.slantRange, state.azimuth, state.elevation};
    }

    const auto positionVector = cartesian::toVector(state.position)
        + cartesian::toVector(velocity) * elapsedSeconds(state.timestamp, time);
    const auto position = cartesian::toPosition(positionVector);
    const auto range = math::Distance::fromMeters(positionVector.norm());
    if (!isFinite(position) || !std::isfinite(range.m()) || range <= 0.0_m)
    {
        return std::nullopt;
    }

    const auto horizontalRange = math::Distance::fromMeters(
        math::Vector{position.x().m(), position.y().m()}.norm());
    return TargetPrediction{
        time,
        range,
        math::Angle::fromRadians(std::atan2(position.y().m(), position.x().m())),
        math::Angle::fromRadians(std::atan2(position.z().m(), horizontalRange.m()))
    };
}

} // namespace target
