#include <beam/config/attitude_config.hpp>

#include <math/tolerance.hpp>

#include <cmath>
#include <stdexcept>
#include <utility>

namespace beam
{
namespace
{
using namespace math::literals;

void validate(
    math::Angle maxAbsPitch,
    const math::SquareMatrix& mountAntToBodyRotation,
    double mountOrthogonalTolerance
) {
    // tolerance를 먼저 검증해야 아래에서 math::Tolerance를 안전하게 만들 수 있다.
    if (!std::isfinite(mountOrthogonalTolerance) || mountOrthogonalTolerance < 0.0)
    {
        throw std::invalid_argument("AttitudeConfig: mountOrthogonalTolerance must be finite and non-negative");
    }

    // NaN은 크기 비교가 모두 false이므로 유한성을 먼저 확인한다.
    if (!maxAbsPitch.isFinite())
    {
        throw std::invalid_argument("AttitudeConfig: maxAbsPitch must be finite");
    }

    if (maxAbsPitch <= 0_deg || maxAbsPitch > 90_deg)
    {
        throw std::invalid_argument("AttitudeConfig: maxAbsPitch must be in (0, 90] degrees");
    }

    if (mountAntToBodyRotation.size() != 3 ||
        !mountAntToBodyRotation.isRotation(math::Tolerance{mountOrthogonalTolerance, 0.0}))
    {
        throw std::invalid_argument("AttitudeConfig: mountAntToBodyRotation must be a 3x3 rotation matrix");
    }
}
} // namespace

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
    validate(_maxAbsPitch, _mountAntToBodyRotation, _mountOrthogonalTolerance);
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

AttitudeConfig AttitudeConfig::Builder::build() const
{
    return AttitudeConfig(_maxAbsPitch, _mountAntToBodyRotation, _mountOrthogonalTolerance);
}

} // namespace beam
