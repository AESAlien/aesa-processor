#include "math/distance.hpp"

#include "relative_comparison.hpp"
#include <cmath>

namespace math
{

Distance Distance::fromMeters(double meters) noexcept
{
    return Distance(meters);
}

Distance Distance::fromKilometers(double kilometers) noexcept
{
    return Distance(kilometers * 1000.0);
}

double Distance::m() const noexcept
{
    return _meters;
}

double Distance::km() const noexcept
{
    return _meters / 1000.0;
}

bool Distance::equals(const Distance& other, const Distance& absoluteTolerance) const noexcept
{
    return std::isfinite(absoluteTolerance._meters)
        && absoluteTolerance._meters >= 0.0
        && std::abs(_meters - other._meters) <= absoluteTolerance._meters;
}

bool Distance::equals(const Distance& other, double relativeTolerance) const noexcept
{
    return detail::relativeEquals(_meters, other._meters, relativeTolerance);
}

bool Distance::operator<(const Distance& other) const noexcept
{
    return _meters < other._meters;
}

bool Distance::operator<=(const Distance& other) const noexcept
{
    return _meters <= other._meters;
}

bool Distance::operator>(const Distance& other) const noexcept
{
    return _meters > other._meters;
}

bool Distance::operator>=(const Distance& other) const noexcept
{
    return _meters >= other._meters;
}

Distance Distance::operator-() const noexcept
{
    return Distance(-_meters);
}

Distance::Distance(double meters) noexcept
    : _meters(meters)
{
}

namespace literals
{

Distance operator""_m(unsigned long long meters) noexcept
{
    return Distance::fromMeters(static_cast<double>(meters));
}

Distance operator""_m(long double meters) noexcept
{
    return Distance::fromMeters(static_cast<double>(meters));
}

Distance operator""_km(unsigned long long kilometers) noexcept
{
    return Distance::fromKilometers(static_cast<double>(kilometers));
}

Distance operator""_km(long double kilometers) noexcept
{
    return Distance::fromKilometers(static_cast<double>(kilometers));
}

}

}
