#include <beam/config/attitude_config.hpp>

#include <math/tolerance.hpp>

#include <stdexcept>
#include <utility>

namespace beam
{

using namespace math::literals;

AttitudeConfig::AttitudeConfig(
    math::Angle maxAbsPitch,
    math::SquareMatrix mountAntToBodyRotation,
    math::Tolerance mountOrthogonalTolerance
) : _maxAbsPitch(maxAbsPitch),
    _mountAntToBodyRotation(std::move(mountAntToBodyRotation)),
    _mountOrthogonalTolerance(std::move(mountOrthogonalTolerance))
{
    validate();
}

const math::Angle& AttitudeConfig::maxAbsPitch() const noexcept
{
    return _maxAbsPitch;
}

const math::SquareMatrix& AttitudeConfig::mountAntToBodyRotation() const noexcept
{
    return _mountAntToBodyRotation;
}

const math::Tolerance& AttitudeConfig::mountOrthogonalTolerance() const noexcept
{
    return _mountOrthogonalTolerance;
}

void AttitudeConfig::validate() const
{
    // 허용오차 없이(NONE_TOLERANCE) 순서 비교로 검사한다
    if (_maxAbsPitch.lessThanOrEqual(0_deg, math::Angle::NONE_TOLERANCE) ||
        _maxAbsPitch.greaterThan(90_deg, math::Angle::NONE_TOLERANCE))
    {
        throw std::invalid_argument("AttitudeConfig: maxAbsPitch must be in (0, 90] degrees");
    }

    if (_mountAntToBodyRotation.size() != 3 ||
        !_mountAntToBodyRotation.isRotation(_mountOrthogonalTolerance))
    {
        throw std::invalid_argument("AttitudeConfig: mountAntToBodyRotation must be a 3x3 rotation matrix");
    }
}

AttitudeConfig AttitudeConfig::Builder::build() const
{
    if (!_maxAbsPitch || !_mountAntToBodyRotation ||
        !_mountOrthogonalTolerance)
    {
        throw std::logic_error("AttitudeConfig::Builder requires every value");
    }

    return AttitudeConfig(
        *_maxAbsPitch,
        *_mountAntToBodyRotation,
        *_mountOrthogonalTolerance);
}

} // namespace beam
