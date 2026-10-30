#include "scheduler/attitude_frame.hpp"

#include "attitude/ant_enu_transform.hpp"

namespace beam
{

AttitudeFrame::AttitudeFrame(
    const AntEnuTransform& transform,
    const SearchGridConfig& searchGridConfig,
    const RequestCoverageConfig& requestCoverageConfig
) : _searchGrid(transform, searchGridConfig),
    _requestCoverage(transform, requestCoverageConfig)
{
}

SearchGrid& AttitudeFrame::searchGrid() noexcept
{
    return _searchGrid;
}

const RequestCoverage& AttitudeFrame::requestCoverage() const noexcept
{
    return _requestCoverage;
}

} // namespace beam
