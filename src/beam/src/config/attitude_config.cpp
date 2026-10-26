#include <beam/config/attitude_config.hpp>

#include <math/tolerance.hpp>

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
    math::Tolerance mountOrthogonalTolerance
) : maxAbsPitch(maxAbsPitch),
    mountAntToBodyRotation(std::move(mountAntToBodyRotation)),
    mountOrthogonalTolerance(std::move(mountOrthogonalTolerance))
{
    validate();
}

void AttitudeConfig::validate() const
{
    // 허용오차 없이(NONE_TOLERANCE) 순서 비교로 검사한다
    if (maxAbsPitch.lessThanOrEqual(0_deg, math::Angle::NONE_TOLERANCE) ||
        maxAbsPitch.greaterThan(90_deg, math::Angle::NONE_TOLERANCE))
    {
        throw std::invalid_argument("AttitudeConfig: maxAbsPitch must be in (0, 90] degrees");
    }

    if (mountAntToBodyRotation.size() != 3 ||
        !mountAntToBodyRotation.isRotation(mountOrthogonalTolerance))
    {
        throw std::invalid_argument("AttitudeConfig: mountAntToBodyRotation must be a 3x3 rotation matrix");
    }
}

AttitudeConfig AttitudeConfig::Builder::build() const
{
    return AttitudeConfig(
        _maxAbsPitch,
        _mountAntToBodyRotation,
        math::Tolerance{
            _mountOrthogonalAbsoluteTolerance,
            _mountOrthogonalRelativeTolerance});
}

} // namespace beam
