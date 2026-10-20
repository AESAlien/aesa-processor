#include <beam/config/attitude_config.hpp>

#include <math/tolerance.hpp>

#include <cmath>
#include <stdexcept>
#include <utility>

namespace aesa_processor::beam
{

AttitudeConfig::AttitudeConfig() : AttitudeConfig(Builder().build())
{
}

AttitudeConfig::AttitudeConfig(
    aesa_processor::math::Angle maxAbsPitch,
    aesa_processor::math::SquareMatrix mountAntToBodyRotation,
    double mountOrthogonalTolerance
) : _maxAbsPitch(maxAbsPitch),
    _mountAntToBodyRotation(std::move(mountAntToBodyRotation)),
    _mountOrthogonalTolerance(mountOrthogonalTolerance)
{
    validate();
}

aesa_processor::math::Angle AttitudeConfig::maxAbsPitch() const noexcept
{
    return _maxAbsPitch;
}

const aesa_processor::math::SquareMatrix& AttitudeConfig::mountAntToBodyRotation() const noexcept
{
    return _mountAntToBodyRotation;
}

double AttitudeConfig::mountOrthogonalTolerance() const noexcept
{
    return _mountOrthogonalTolerance;
}

void AttitudeConfig::validate() const
{
    // tolerance를 먼저 검증해야 아래에서 aesa_processor::math::Tolerance를 안전하게 만들 수 있다.
    if (!std::isfinite(_mountOrthogonalTolerance) || _mountOrthogonalTolerance < 0.0)
    {
        throw std::invalid_argument("AttitudeConfig: mountOrthogonalTolerance must be finite and non-negative");
    }

    // 허용오차 없이(NONE_TOLERANCE) 순서 비교로 검사한다
    if (_maxAbsPitch.lessThanOrEqual(0_deg, aesa_processor::math::Angle::NONE_TOLERANCE) ||
        _maxAbsPitch.greaterThan(90_deg, aesa_processor::math::Angle::NONE_TOLERANCE))
    {
        throw std::invalid_argument("AttitudeConfig: maxAbsPitch must be in (0, 90] degrees");
    }

    if (_mountAntToBodyRotation.size() != 3 ||
        !_mountAntToBodyRotation.isRotation(aesa_processor::math::Tolerance{_mountOrthogonalTolerance, 0.0}))
    {
        throw std::invalid_argument("AttitudeConfig: mountAntToBodyRotation must be a 3x3 rotation matrix");
    }
}

AttitudeConfig AttitudeConfig::Builder::build() const
{
    return AttitudeConfig(_maxAbsPitch, _mountAntToBodyRotation, _mountOrthogonalTolerance);
}

} // namespace aesa_processor::beam
