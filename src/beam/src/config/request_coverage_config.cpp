#include <beam/config/request_coverage_config.hpp>

#include <stdexcept>

namespace beam
{

using namespace math::literals;

RequestCoverageConfig::RequestCoverageConfig(
    math::Angle azimuthMin,
    math::Angle azimuthMax,
    math::Elevation elevationMin,
    math::Elevation elevationMax
) : _azimuthMin(azimuthMin),
    _azimuthMax(azimuthMax),
    _elevationMin(elevationMin),
    _elevationMax(elevationMax)
{
    validate();
}

const math::Angle& RequestCoverageConfig::azimuthMin() const noexcept
{
    return _azimuthMin;
}

const math::Angle& RequestCoverageConfig::azimuthMax() const noexcept
{
    return _azimuthMax;
}

const math::Elevation& RequestCoverageConfig::elevationMin() const noexcept
{
    return _elevationMin;
}

const math::Elevation& RequestCoverageConfig::elevationMax() const noexcept
{
    return _elevationMax;
}

void RequestCoverageConfig::validate() const
{
    // 허용오차 없이(NONE_TOLERANCE) 순서 비교로 검사한다
    if (_azimuthMin.lessThan(-180_deg, math::Angle::NONE_TOLERANCE) ||
        _azimuthMax.greaterThan(180_deg, math::Angle::NONE_TOLERANCE))
    {
        throw std::invalid_argument("RequestCoverageConfig: azimuth must be in [-180, 180] degrees");
    }

    if (_azimuthMin.greaterThanOrEqual(_azimuthMax, math::Angle::NONE_TOLERANCE))
    {
        throw std::invalid_argument("RequestCoverageConfig: azimuthMin must be less than azimuthMax");
    }

    if (_elevationMin.deg() >= _elevationMax.deg())
    {
        throw std::invalid_argument("RequestCoverageConfig: elevationMin must be less than elevationMax");
    }
}

RequestCoverageConfig RequestCoverageConfig::Builder::build() const
{
    if (!_azimuthMin || !_azimuthMax || !_elevationMin || !_elevationMax)
    {
        throw std::logic_error("RequestCoverageConfig::Builder requires every value");
    }

    return RequestCoverageConfig(
        *_azimuthMin,
        *_azimuthMax,
        *_elevationMin,
        *_elevationMax);
}

} // namespace beam
