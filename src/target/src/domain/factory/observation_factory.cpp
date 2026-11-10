#include "observation_factory.hpp"

namespace target
{

std::vector<Observation> ObservationFactory::from(const DetectionEvent& detection)
{
    std::vector<Observation> observations;
    observations.reserve(detection.plots.size());
    for (const auto& plot : detection.plots)
    {
        observations.emplace_back(
            math::SphericalPosition(
                plot.slantRange,
                math::SphericalDirection(plot.azimuth, plot.elevation)
            ),
            plot.dopplerVelocity,
            plot.power_db,
            detection.transmitTime
        );
    }
    return observations;
}

} // namespace target
