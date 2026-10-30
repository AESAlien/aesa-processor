#include "scheduler/request_coverage.hpp"

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
    return _config.azimuthRange().contains(direction_ant.azimuth)
        && _config.elevationRange().contains(direction_ant.elevation);
}

} // namespace beam
