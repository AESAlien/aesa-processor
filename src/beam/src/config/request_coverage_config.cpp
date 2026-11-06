#include <beam/config/request_coverage_config.hpp>

#include <stdexcept>

namespace beam
{

using namespace math::literals;

RequestCoverageConfig::RequestCoverageConfig(
    math::CircularAngleRange azimuthRange,
    math::AngleRange elevationRange
) : _azimuthRange(azimuthRange),
    _elevationRange(elevationRange)
{
    validate();
}

const math::CircularAngleRange& RequestCoverageConfig::azimuthRange() const noexcept
{
    return _azimuthRange;
}

const math::AngleRange& RequestCoverageConfig::elevationRange() const noexcept
{
    return _elevationRange;
}

void RequestCoverageConfig::validate() const
{
    // 허용오차 없이(NONE_TOLERANCE) 순서 비교로 검사한다
    if (_elevationRange.start().lessThan(-90_deg, math::Angle::NONE_TOLERANCE) ||
        _elevationRange.end().greaterThan(90_deg, math::Angle::NONE_TOLERANCE))
    {
        throw std::invalid_argument("RequestCoverageConfig: elevation must be in [-90, 90] degrees");
    }
}

RequestCoverageConfig RequestCoverageConfig::Builder::build() const
{
    if (!_azimuthRange || !_elevationRange)
    {
        throw std::logic_error("RequestCoverageConfig::Builder requires every value");
    }

    return RequestCoverageConfig(
        *_azimuthRange,
        *_elevationRange);
}

} // namespace beam
