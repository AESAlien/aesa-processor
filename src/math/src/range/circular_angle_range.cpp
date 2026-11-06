#include "math/range/circular_angle_range.hpp"

#include <cmath>
#include <stdexcept>

namespace math
{
using namespace literals;

namespace
{

bool isFullTurn(Angle start, Angle end)
{
    const double difference = end.deg() - start.deg();
    if (!std::isfinite(difference) || difference == 0.0 || std::abs(difference) > 360.0)
    {
        throw std::invalid_argument("CircularAngleRange: boundary difference must be nonzero and within one turn");
    }
    return std::abs(difference) == 360.0;
}

}

CircularAngleRange::CircularAngleRange(CircularAngle start, CircularAngle end)
    : CircularAngleRange(start, end, false)
{
}

CircularAngleRange::CircularAngleRange(Angle start, Angle end)
    : CircularAngleRange(CircularAngle(start), CircularAngle(end), isFullTurn(start, end))
{
}

CircularAngleRange::CircularAngleRange(CircularAngle start, CircularAngle end, bool fullCircle)
    : _start(start),
      _end(end),
      _sweep(fullCircle ? 360_deg : wrap360(end.angle() - start.angle()))
{
    if (_sweep.deg() == 0.0)
    {
        throw std::invalid_argument("CircularAngleRange: boundaries must represent different directions");
    }
}

CircularAngleRange CircularAngleRange::fullCircle()
{
    return CircularAngleRange(CircularAngle(0_deg), CircularAngle(0_deg), true);
}

const CircularAngle& CircularAngleRange::start() const noexcept
{
    return _start;
}

const CircularAngle& CircularAngleRange::end() const noexcept
{
    return _end;
}

bool CircularAngleRange::isFullCircle() const noexcept
{
    return _sweep.deg() == 360.0;
}

bool CircularAngleRange::contains(
    CircularAngle value,
    const QuantityTolerance<Angle>& tolerance
) const noexcept
{
    const Angle offset = wrap360(value.angle() - _start.angle());
    return isFullCircle()
        || offset.lessThanOrEqual(_sweep, Angle::NONE_TOLERANCE)
        || value.isEquivalentTo(_start, tolerance)
        || value.isEquivalentTo(_end, tolerance);
}

bool CircularAngleRange::contains(
    Angle value,
    const QuantityTolerance<Angle>& tolerance
) const noexcept
{
    return contains(CircularAngle(value), tolerance);
}

}
