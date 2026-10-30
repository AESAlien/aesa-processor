#include "scheduler/request_coverage.hpp"

#include <math/quantity/angle.hpp>

namespace beam
{

RequestCoverage::RequestCoverage(const AntEnuTransform& transform, const RequestCoverageConfig& config)
    : _transform(transform),
      _config(config)
{
}

bool RequestCoverage::contains(const math::SphericalDirection& direction_enu) const
{
    const math::SphericalDirection direction_ant = _transform.enuToAnt(direction_enu);
    // Azimuth는 [0, 360)으로 정규화되어 있으므로 음수 경계와 비교할 수 있게 [-180, 180)으로 바꾼다.
    const math::Angle azimuth_ant = math::wrap180(math::Angle::fromDegrees(direction_ant.azimuth.deg()));
    const math::Angle elevation_ant = math::Angle::fromDegrees(direction_ant.elevation.deg());
    const math::Angle elevationMin = math::Angle::fromDegrees(_config.elevationMin().deg());
    const math::Angle elevationMax = math::Angle::fromDegrees(_config.elevationMax().deg());

    return azimuth_ant >= _config.azimuthMin() &&
           azimuth_ant <= _config.azimuthMax() &&
           elevation_ant >= elevationMin &&
           elevation_ant <= elevationMax;
}

} // namespace beam
