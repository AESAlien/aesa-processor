#include "scheduler/request_coverage.hpp"

#include <math/quantity/angle.hpp>

namespace beam
{

namespace
{

// 각도가 [min, max] 안이면 true. 경계는 기본 허용오차 안이면 포함한다.
bool isWithin(const math::Angle& value, const math::Angle& min, const math::Angle& max)
{
    return value >= min && value <= max;
}

} // namespace

using namespace math::literals;

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

    // -180°와 180°는 같은 방향이다. wrap180은 180°를 -180°로 바꾸므로
    // azimuthMax가 180°인 영역에서도 판정되도록 +360°한 값도 함께 비교한다.
    // 설정이 [-180, 180]으로 검증되어 있어, 이 보정이 영향을 주는 값은 -180° 근처뿐이다.
    const bool azimuthInside =
        isWithin(azimuth_ant, _config.azimuthMin(), _config.azimuthMax()) ||
        isWithin(azimuth_ant + 360_deg, _config.azimuthMin(), _config.azimuthMax());

    return azimuthInside && isWithin(elevation_ant, elevationMin, elevationMax);
}

} // namespace beam
