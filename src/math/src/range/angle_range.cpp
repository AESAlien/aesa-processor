#include "math/range/angle_range.hpp"

#include <stdexcept>

namespace math
{

AngleRange::AngleRange(Angle start, Angle end)
    : _start(start),
      _end(end)
{
    if (_start.greaterThanOrEqual(_end, Angle::NONE_TOLERANCE))
    {
        throw std::invalid_argument("AngleRange: start must be less than end");
    }
}

AngleRange::AngleRange(TiltAngle start, TiltAngle end)
    : AngleRange(Angle::fromDegrees(start.deg()), Angle::fromDegrees(end.deg()))
{
}

const Angle& AngleRange::start() const noexcept
{
    return _start;
}

const Angle& AngleRange::end() const noexcept
{
    return _end;
}

bool AngleRange::contains(
    Angle value,
    const QuantityTolerance<Angle>& tolerance
) const noexcept
{
    return value.greaterThanOrEqual(_start, tolerance)
        && value.lessThanOrEqual(_end, tolerance);
}

bool AngleRange::contains(
    TiltAngle value,
    const QuantityTolerance<Angle>& tolerance
) const noexcept
{
    return contains(Angle::fromDegrees(value.deg()), tolerance);
}

}
