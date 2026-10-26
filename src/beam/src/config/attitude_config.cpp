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
) : maxAbsPitch(maxAbsPitch),
    mountAntToBodyRotation(std::move(mountAntToBodyRotation)),
    mountOrthogonalTolerance(mountOrthogonalTolerance)
{
    validate();
}

void AttitudeConfig::validate() const
{
    // tolerance를 먼저 검증해야 아래에서 math::Tolerance를 안전하게 만들 수 있다.
    if (!std::isfinite(mountOrthogonalTolerance) || mountOrthogonalTolerance < 0.0)
    {
        throw std::invalid_argument("AttitudeConfig: mountOrthogonalTolerance must be finite and non-negative");
    }

    // 허용오차 없이(NONE_TOLERANCE) 순서 비교로 검사한다
    if (maxAbsPitch.lessThanOrEqual(0_deg, math::Angle::NONE_TOLERANCE) ||
        maxAbsPitch.greaterThan(90_deg, math::Angle::NONE_TOLERANCE))
    {
        throw std::invalid_argument("AttitudeConfig: maxAbsPitch must be in (0, 90] degrees");
    }

    if (mountAntToBodyRotation.size() != 3 ||
        !mountAntToBodyRotation.isRotation(math::Tolerance{mountOrthogonalTolerance, 0.0}))
    {
        throw std::invalid_argument("AttitudeConfig: mountAntToBodyRotation must be a 3x3 rotation matrix");
    }
}

AttitudeConfig AttitudeConfig::Builder::build() const
{
    return AttitudeConfig(_maxAbsPitch, _mountAntToBodyRotation, _mountOrthogonalTolerance);
}

} // namespace beam
