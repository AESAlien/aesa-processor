#include "detection_matcher.hpp"

#include "domain/tracking_state.hpp"

#include <cmath>

namespace target
{
namespace
{

bool hasValidGeometry(
    const math::Distance& range,
    const math::Angle& azimuth,
    const math::Angle& elevation)
{
    return std::isfinite(range.m()) && range.m() > 0.0
        && std::isfinite(azimuth.deg()) && std::isfinite(elevation.deg())
        && std::abs(elevation.deg()) <= 90.0;
}

bool positionsEqual(
    const DetectionEvent::Plot& plot,
    const math::Distance& range,
    const math::Angle& azimuth,
    const math::Angle& elevation)
{
    // Preserve the current exact antenna-position policy until NDS is specified.
    return plot.slantRange.m() == range.m()
        && plot.azimuth.deg() == azimuth.deg()
        && plot.elevation.deg() == elevation.deg();
}

} // namespace

bool DetectionMatcher::isValidPosition(const DetectionEvent::Plot& plot) const
{
    return hasValidGeometry(plot.slantRange, plot.azimuth, plot.elevation)
        && std::isfinite(plot.dopplerVelocity.mps());
}

bool DetectionMatcher::matchesPosition(
    const DetectionEvent::Plot& plot,
    const DetectionEvent::Plot& other) const
{
    return isValidPosition(plot) && isValidPosition(other)
        && positionsEqual(plot, other.slantRange, other.azimuth, other.elevation);
}

bool DetectionMatcher::matchesPosition(
    const DetectionEvent::Plot& plot,
    const TargetPrediction& prediction) const
{
    return isValidPosition(plot)
        && hasValidGeometry(prediction.slantRange, prediction.azimuth, prediction.elevation)
        && positionsEqual(plot, prediction.slantRange, prediction.azimuth, prediction.elevation);
}

} // namespace target
