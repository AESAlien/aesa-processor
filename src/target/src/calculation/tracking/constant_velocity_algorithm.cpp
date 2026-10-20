#include "constant_velocity_algorithm.hpp"

#include "domain/target.hpp"

#include <math/geometry/cartesian_displacement.hpp>
#include <math/geometry/cartesian_position.hpp>
#include <math/geometry/cartesian_velocity.hpp>
#include <math/geometry/spherical_position.hpp>

#include <stdexcept>

namespace aesa_processor::target
{
namespace
{

using namespace aesa_processor::math::literals;

double elapsedSeconds(
    std::chrono::milliseconds previous,
    std::chrono::milliseconds current)
{
    // Validated nonnegative, ordered times keep the integer difference in range.
    // Subtract before conversion to preserve short intervals at large timestamps.
    return std::chrono::duration<double>(current - previous).count();
}

std::optional<TargetTrackingState> estimateTrackingState(
    const aesa_processor::math::CartesianPosition& previousPosition,
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
    if (velocity.x.equals(0.0_mps, aesa_processor::math::LinearVelocity::NONE_TOLERANCE)
        && velocity.y.equals(0.0_mps, aesa_processor::math::LinearVelocity::NONE_TOLERANCE)
        && velocity.z.equals(0.0_mps, aesa_processor::math::LinearVelocity::NONE_TOLERANCE))
    {
        return TargetPrediction{time, state.sphericalPosition};
    }

    try
    {
        const auto displacement = velocity * (time - state.timestamp);
        const auto predictedPosition = state.position + displacement;
        return TargetPrediction{time, predictedPosition.toSpherical()};
    }
    catch (const std::invalid_argument&)
    {
        return std::nullopt;
    }
    catch (const std::overflow_error&)
    {
        return std::nullopt;
    }
}

} // namespace aesa_processor::target
