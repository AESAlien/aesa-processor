#include <beam/config/attitude_config.hpp>

#include <math/tolerance.hpp>

#include <cmath>
#include <stdexcept>
#include <utility>

namespace beam
{

AttitudeConfig::AttitudeConfig() : AttitudeConfig(Builder().build())
{
}

AttitudeConfig::AttitudeConfig(
    math::Angle maxAbsPitch,
    math::SquareMatrix mountAntToBodyRotation,
    double mountOrthogonalTolerance
) : _maxAbsPitch(maxAbsPitch),
    _mountAntToBodyRotation(std::move(mountAntToBodyRotation)),
    _mountOrthogonalTolerance(mountOrthogonalTolerance)
{
    validate();
}

math::Angle AttitudeConfig::maxAbsPitch() const noexcept
{
    return _maxAbsPitch;
}

const math::SquareMatrix& AttitudeConfig::mountAntToBodyRotation() const noexcept
{
    return _mountAntToBodyRotation;
}

double AttitudeConfig::mountOrthogonalTolerance() const noexcept
{
    return _mountOrthogonalTolerance;
}

void AttitudeConfig::validate() const
{
    // tolerance를 먼저 검증해야 아래에서 math::Tolerance를 안전하게 만들 수 있다.
    if (!std::isfinite(_mountOrthogonalTolerance) || _mountOrthogonalTolerance < 0.0)
    {
        throw std::invalid_argument("AttitudeConfig: mountOrthogonalTolerance must be finite and non-negative");
    }

    // Angle의 순서 비교는 허용오차를 적용하므로, (0, 90] 불변식은 주기 보정 없는 수치(deg())로 직접 검사한다.
    const double maxAbsPitchDeg = _maxAbsPitch.deg();
    if (maxAbsPitchDeg <= 0.0 || maxAbsPitchDeg > 90.0)
    {
        throw std::invalid_argument("AttitudeConfig: maxAbsPitch must be in (0, 90] degrees");
    }

    if (_mountAntToBodyRotation.size() != 3 ||
        !_mountAntToBodyRotation.isRotation(math::Tolerance{_mountOrthogonalTolerance, 0.0}))
    {
        throw std::invalid_argument("AttitudeConfig: mountAntToBodyRotation must be a 3x3 rotation matrix");
    }
}

AttitudeConfig AttitudeConfig::Builder::build() const
{
    return AttitudeConfig(_maxAbsPitch, _mountAntToBodyRotation, _mountOrthogonalTolerance);
}

} // namespace beam
