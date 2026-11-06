#pragma once

#include "attitude/ant_enu_transform.hpp"

#include <beam/config/request_coverage_config.hpp>

#include <math/geometry/spherical_direction.hpp>

namespace beam
{

// 요청 빔(확인·추적)의 ENU 방향이 추적 영역(안테나 기준) 안에 있는지 판정한다.
// 자세정보 하나에 묶인 변환을 가지므로, 자세정보가 바뀌면 새로 만들어야 한다.
// 빔 중심 방향만 비교하며 빔 폭은 고려하지 않는다.
class RequestCoverage
{
public:
    RequestCoverage(const AntEnuTransform& transform, const RequestCoverageConfig& config);

    // ENU 방향을 안테나 기준으로 바꾼 뒤 원형 방위각 구간과 고각 구간에 포함되는지 검사한다.
    // 경계는 기본 허용오차(Angle::DEFAULT_TOLERANCE) 안이면 영역에 포함한다.
    bool contains(const math::SphericalDirection& direction_enu) const;

private:
    AntEnuTransform _transform;
    RequestCoverageConfig _config;
};

} // namespace beam
