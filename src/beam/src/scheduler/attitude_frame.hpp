#pragma once

#include "scheduler/request_coverage.hpp"
#include "scheduler/search_grid.hpp"

#include <beam/config/request_coverage_config.hpp>
#include <beam/config/search_grid_config.hpp>

namespace beam
{

// 자세정보 하나로 만든 탐색 격자와 요청 빔 영역 판정을 묶는다.
// 둘을 같은 변환으로만 만들 수 있게 해서, 격자와 영역 판정의 자세가 어긋나거나 한쪽만 있는 상태를 막는다.
class AttitudeFrame
{
public:
    AttitudeFrame(
        const AntEnuTransform& transform,
        const SearchGridConfig& searchGridConfig,
        const RequestCoverageConfig& requestCoverageConfig);

    // 탐색 빔을 꺼낼 때 커서가 움직이므로 수정 가능한 참조를 반환한다.
    SearchGrid& searchGrid() noexcept;
    const RequestCoverage& requestCoverage() const noexcept;

private:
    SearchGrid _searchGrid;
    RequestCoverage _requestCoverage;
};

} // namespace beam
