#include "constant_velocity_algorithm.hpp"

#include "domain/confirmed_track.hpp"

#include <math/geometry/cartesian_displacement.hpp>
#include <math/geometry/cartesian_position.hpp>
#include <math/geometry/cartesian_velocity.hpp>
#include <math/geometry/spherical_position.hpp>

#include <stdexcept>
#include <utility>

namespace target
{

std::optional<TrackingResult> ConstantVelocityAlgorithm::initialize(
    const Observation& first,
    const Observation& second
) const {
    const auto firstTime = first.observationTime();
    const auto secondTime = second.observationTime();
    if (secondTime <= firstTime)
    {
        return std::nullopt;
    }

    const auto firstPosition = first.sphericalPosition().toCartesian();
    const auto secondPosition = second.sphericalPosition().toCartesian();
    // 시각이 큰 경우에도 짧은 간격을 보존하도록 먼저 뺀 뒤 초 단위로 변환한다.
    const auto elapsed = std::chrono::duration<double>(secondTime - firstTime);
    const auto velocity = (secondPosition - firstPosition) / elapsed;

    auto state = TrackState(
        second.sphericalPosition(),
        second.dopplerVelocity(),
        second.power_db(),
        secondTime,
        secondPosition,
        velocity
    );
    return TrackingResult{std::move(state), {}};
}

std::optional<TrackingResult> ConstantVelocityAlgorithm::update(
    const ConfirmedTrack& current,
    const Observation& observation
) const {
    const auto& currentState = current.state();
    const auto previousTime = currentState->timestamp();
    const auto time = observation.observationTime();
    if (time <= previousTime)
    {
        return std::nullopt;
    }

    const auto position = observation.sphericalPosition().toCartesian();
    // 시각이 큰 경우에도 짧은 간격을 보존하도록 먼저 뺀 뒤 초 단위로 변환한다.
    const auto elapsed = std::chrono::duration<double>(time - previousTime);
    const auto velocity = (position - currentState->position()) / elapsed;

    auto state = TrackState(
        observation.sphericalPosition(),
        observation.dopplerVelocity(),
        observation.power_db(),
        time,
        position,
        velocity
    );
    return TrackingResult{std::move(state), {}};
}

std::optional<TrackPrediction> ConstantVelocityAlgorithm::predict(
    const ConfirmedTrack& current,
    std::chrono::milliseconds time
) const {
    using namespace math::literals;

    const auto& currentState = current.state();
    if (!currentState || time < currentState->timestamp())
    {
        return std::nullopt;
    }

    const auto& state = *currentState;
    const auto& velocity = state.velocity();
    if (velocity.x == 0_mps && velocity.y == 0_mps && velocity.z == 0_mps)
    {
        // 정지 상태에서는 좌표 변환 오차 없이 관측 위치를 보존한다.
        return TrackPrediction(time, state.sphericalPosition());
    }

    try
    {
        const auto displacement = velocity * (time - state.timestamp());
        const auto predictedPosition = state.position() + displacement;
        return TrackPrediction(time, predictedPosition.toSpherical());
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

} // namespace target
