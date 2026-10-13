#include "math/trigonometry.hpp"

#include <cmath>
#include <stdexcept>

namespace math
{
namespace
{
void requireFinite(Angle angle)
{
    if (!angle.isFinite())
    {
        throw std::domain_error("Trigonometric functions require a finite angle");
    }
}

void requireUnitInterval(double value)
{
    if (!std::isfinite(value) || value < -1.0 || value > 1.0)
    {
        throw std::domain_error("Inverse sine and cosine require a finite value in [-1, 1]");
    }
}
}

double sin(Angle angle)
{
    requireFinite(angle);
    return std::sin(angle.wrap180().rad());
}

double cos(Angle angle)
{
    requireFinite(angle);
    return std::cos(angle.wrap180().rad());
}

double tan(Angle angle)
{
    requireFinite(angle);
    return std::tan(angle.wrap180().rad());
}

Angle asin(double value)
{
    requireUnitInterval(value);
    return Angle::fromRadians(std::asin(value));
}

Angle acos(double value)
{
    requireUnitInterval(value);
    return Angle::fromRadians(std::acos(value));
}

Angle atan(double value)
{
    if (!std::isfinite(value))
    {
        throw std::domain_error("Inverse tangent requires a finite value");
    }
    return Angle::fromRadians(std::atan(value));
}

}
