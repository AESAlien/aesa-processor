#include "math/trigonometry.hpp"

#include <cmath>
#include <stdexcept>

namespace math
{

double sin(Angle angle)
{
    return std::sin(wrap180(angle).rad());
}

double cos(Angle angle)
{
    return std::cos(wrap180(angle).rad());
}

double tan(Angle angle)
{
    return std::tan(wrap180(angle).rad());
}

Angle asin(double value)
{
    if (!std::isfinite(value) || value < -1.0 || value > 1.0)
    {
        throw std::domain_error("Inverse sine requires a finite value in [-1, 1]");
    }

    return Angle::fromRadians(std::asin(value));
}

Angle acos(double value)
{
    if (!std::isfinite(value) || value < -1.0 || value > 1.0)
    {
        throw std::domain_error("Inverse cosine requires a finite value in [-1, 1]");
    }

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

Angle atan2(Length y, Length x)
{
    return Angle::fromRadians(std::atan2(y.m(), x.m()));
}

}
