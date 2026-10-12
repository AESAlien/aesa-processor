#include "constant_velocity_algorithm.hpp"

#include <array>
#include <cmath>

namespace target
{
namespace
{

bool isValidPlot(const DetectionEvent::Plot& plot)
{
    return std::isfinite(plot.slantRange.m()) && plot.slantRange.m() > 0.0
        && std::isfinite(plot.azimuth.deg())
        && std::isfinite(plot.elevation.deg())
        && std::abs(plot.elevation.deg()) <= 90.0
        && std::isfinite(plot.dopplerVelocity.mps());
}

std::array<double, 3> cartesianPosition(const DetectionEvent::Plot& plot)
{
    const double range = plot.slantRange.m();
    const double azimuth = plot.azimuth.rad();
    const double elevation = plot.elevation.rad();
    const double horizontalRange = range * std::cos(elevation);
    return {
        horizontalRange * std::cos(azimuth),
        horizontalRange * std::sin(azimuth),
        range * std::sin(elevation)
    };
}

double elapsedSeconds(
    std::chrono::milliseconds previous,
    std::chrono::milliseconds current)
{
    // Callers validate nonnegative, ordered times, so the integer difference fits.
    // Subtract before conversion to preserve short intervals at large timestamps.
    return std::chrono::duration<double>(current - previous).count();
}

bool isFinite(const std::array<double, 3>& values)
{
    return std::isfinite(values[0]) && std::isfinite(values[1])
        && std::isfinite(values[2]);
}

bool isValidState(const TargetTrackingState& state)
{
    const DetectionEvent::Plot plot{state.slantRange, state.azimuth, state.elevation,
        state.dopplerVelocity, state.powerDb};
    const auto& velocity = state.velocityMetersPerSecond;
    return state.timestamp.count() >= 0 && isValidPlot(plot)
        && isFinite(state.positionMeters) && isFinite(velocity)
        && std::isfinite(std::hypot(velocity[0], velocity[1], velocity[2]));
}

} // namespace

std::optional<TargetTrackingState> ConstantVelocityAlgorithm::initialize(
    const DetectionEvent::Plot& first,
    std::chrono::milliseconds firstTime,
    const DetectionEvent::Plot& second,
    std::chrono::milliseconds secondTime) const
{
    if (!isValidPlot(first) || !isValidPlot(second)
        || firstTime.count() < 0 || secondTime <= firstTime)
    {
        return std::nullopt;
    }

    const auto firstPosition = cartesianPosition(first);
    const auto secondPosition = cartesianPosition(second);
    const double seconds = elapsedSeconds(firstTime, secondTime);
    std::array<double, 3> velocity{};
    for (std::size_t index = 0; index < velocity.size(); ++index)
    {
        velocity[index] = (secondPosition[index] - firstPosition[index]) / seconds;
    }

    if (!isFinite(firstPosition) || !isFinite(secondPosition) || !isFinite(velocity)
        || !std::isfinite(std::hypot(velocity[0], velocity[1], velocity[2])))
    {
        return std::nullopt;
    }

    return TargetTrackingState{second.slantRange, second.azimuth, second.elevation,
        second.dopplerVelocity, second.power_db, secondTime, secondPosition, velocity};
}

std::optional<TargetTrackingState> ConstantVelocityAlgorithm::update(
    const TargetTrackingState& state,
    const DetectionEvent::Plot& plot,
    std::chrono::milliseconds time) const
{
    if (!isValidState(state) || !isValidPlot(plot) || time <= state.timestamp)
    {
        return std::nullopt;
    }

    const auto position = cartesianPosition(plot);
    const double seconds = elapsedSeconds(state.timestamp, time);
    std::array<double, 3> velocity{};
    for (std::size_t index = 0; index < velocity.size(); ++index)
    {
        velocity[index] = (position[index] - state.positionMeters[index]) / seconds;
    }

    if (!isFinite(position) || !isFinite(velocity)
        || !std::isfinite(std::hypot(velocity[0], velocity[1], velocity[2])))
    {
        return std::nullopt;
    }

    return TargetTrackingState{plot.slantRange, plot.azimuth, plot.elevation,
        plot.dopplerVelocity, plot.power_db, time, position, velocity};
}

std::optional<TargetPrediction> ConstantVelocityAlgorithm::predict(
    const TargetTrackingState& state,
    std::chrono::milliseconds time) const
{
    if (!isValidState(state) || time < state.timestamp)
    {
        return std::nullopt;
    }

    // Preserve a stationary measured position exactly while matching remains exact.
    const auto& velocity = state.velocityMetersPerSecond;
    if (velocity[0] == 0.0 && velocity[1] == 0.0 && velocity[2] == 0.0)
    {
        return TargetPrediction{time, state.slantRange, state.azimuth, state.elevation};
    }

    const double seconds = elapsedSeconds(state.timestamp, time);
    auto position = state.positionMeters;
    for (std::size_t index = 0; index < position.size(); ++index)
    {
        position[index] += velocity[index] * seconds;
    }

    const double range = std::hypot(position[0], position[1], position[2]);
    if (!isFinite(position) || !std::isfinite(range) || range <= 0.0)
    {
        return std::nullopt;
    }

    const double azimuth = std::atan2(position[1], position[0]);
    const double elevation = std::atan2(position[2], std::hypot(position[0], position[1]));
    return TargetPrediction{
        time,
        math::Distance::fromMeters(range),
        math::Angle::fromRadians(azimuth),
        math::Angle::fromRadians(elevation)
    };
}

} // namespace target
