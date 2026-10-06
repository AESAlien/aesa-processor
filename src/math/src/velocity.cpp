#include "math/velocity.hpp"

#include "relative_comparison.hpp"
#include <cmath>

namespace math
{

Velocity Velocity::fromMetersPerSecond(double metersPerSecond) noexcept
{
    return Velocity(metersPerSecond);
}

double Velocity::mps() const noexcept
{
    return _metersPerSecond;
}

bool Velocity::equals(const Velocity& other, const Velocity& absoluteTolerance) const noexcept
{
    return std::isfinite(absoluteTolerance._metersPerSecond)
        && absoluteTolerance._metersPerSecond >= 0.0
        && std::abs(_metersPerSecond - other._metersPerSecond) <= absoluteTolerance._metersPerSecond;
}

bool Velocity::equals(const Velocity& other, double relativeTolerance) const noexcept
{
    return detail::relativeEquals(_metersPerSecond, other._metersPerSecond, relativeTolerance);
}

bool Velocity::operator<(const Velocity& other) const noexcept
{
    return _metersPerSecond < other._metersPerSecond;
}

bool Velocity::operator<=(const Velocity& other) const noexcept
{
    return _metersPerSecond <= other._metersPerSecond;
}

bool Velocity::operator>(const Velocity& other) const noexcept
{
    return _metersPerSecond > other._metersPerSecond;
}

bool Velocity::operator>=(const Velocity& other) const noexcept
{
    return _metersPerSecond >= other._metersPerSecond;
}

Velocity Velocity::operator-() const noexcept
{
    return Velocity(-_metersPerSecond);
}

Velocity::Velocity(double metersPerSecond) noexcept
    : _metersPerSecond(metersPerSecond)
{
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
