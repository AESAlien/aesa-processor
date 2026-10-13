#include "detection_matcher.hpp"

#include "calculation/tracking/cartesian_math.hpp"
#include "domain/target.hpp"
#include "validation/detection_validation.hpp"

#include <chrono>
#include <cmath>

namespace target
{
namespace
{

using namespace math::literals;

const math::Velocity MAX_INITIAL_SPEED = 1000.0_mps;
const math::Velocity MAX_INITIAL_DOPPLER_DIFFERENCE = 30.0_mps;

bool positionsEqual(
    const Target& observation,
    const math::Distance& range,
    const math::Angle& azimuth,
    const math::Angle& elevation)
{
    // Preserve the current exact antenna-position policy until NDS is specified.
    return observation.slantRange().equals(range, math::Tolerance{0.0_m, 0.0})
        && observation.azimuth().equals(azimuth, math::Tolerance{0.0_deg, 0.0})
        && observation.elevation().equals(elevation, math::Tolerance{0.0_deg, 0.0});
}

} // namespace

bool DetectionMatcher::isValidPosition(const Target& observation) const
{
    return validation::isValidTarget(observation);
}

bool DetectionMatcher::matchesPosition(
    const Target& observation,
    const Target& other) const
{
    return isValidPosition(observation) && isValidPosition(other)
        && positionsEqual(observation, other.slantRange(), other.azimuth(), other.elevation());
}

bool DetectionMatcher::matchesConfirmation(
    const Target& observation,
    const Target& tentative) const
{
    const auto time = observation.lastObservationTime();
    if (tentative.state() != TargetState::TENTATIVE
        || !isValidPosition(observation) || !isValidPosition(tentative)
        || time <= tentative.lastResultTime() || time <= tentative.initialTime())
    {
        return false;
    }

    const auto displacement = cartesian::toVector(cartesian::position(observation))
        - cartesian::toVector(cartesian::position(tentative));
    // Ordered nonnegative times keep this integer subtraction within its range.
    const double seconds = std::chrono::duration<double>(time - tentative.initialTime()).count();
    const auto speed = math::Velocity::fromMetersPerSecond((displacement / seconds).norm());
    return std::isfinite(speed.mps()) && speed <= MAX_INITIAL_SPEED
        && observation.dopplerVelocity().equals(
            tentative.dopplerVelocity(), math::Tolerance{MAX_INITIAL_DOPPLER_DIFFERENCE, 0.0});
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
