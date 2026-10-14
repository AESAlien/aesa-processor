#include "math/quantity/velocity.hpp"

#include <cmath>

namespace math
{
using namespace literals;

const QuantityTolerance<Velocity> Velocity::NONE_TOLERANCE{
    Velocity::fromMetersPerSecond(0.0),
    0.0
};

const QuantityTolerance<Velocity> Velocity::DEFAULT_TOLERANCE{
    Velocity::fromMetersPerSecond(DEFAULT_ABSOLUTE_TOLERANCE),
    DEFAULT_RELATIVE_TOLERANCE
};

Velocity Velocity::fromMetersPerSecond(double metersPerSecond) noexcept
{
    return Velocity(metersPerSecond);
}

double Velocity::mps() const noexcept
{
    return _value;
}

Velocity Velocity::operator-() const noexcept
{
    return Velocity(-_value);
}

Velocity::Velocity(double metersPerSecond) noexcept
    : Quantity<Velocity>(metersPerSecond)
{
}

Velocity abs(Velocity velocity) noexcept
{
    return Velocity::fromMetersPerSecond(std::abs(velocity.mps()));
}

namespace literals
{

Velocity operator""_mps(unsigned long long metersPerSecond) noexcept
{
    return Velocity::fromMetersPerSecond(static_cast<double>(metersPerSecond));
}

Velocity operator""_mps(long double metersPerSecond) noexcept
{
    return Velocity::fromMetersPerSecond(static_cast<double>(metersPerSecond));
}

}

}
