#include "math/distance.hpp"

namespace math
{

Distance::Distance(double kilometers) noexcept : _kilometers(kilometers)
{
}

Distance Distance::fromKilometers(double kilometers) noexcept
{
    return Distance(kilometers);
}

double Distance::km() const noexcept
{
    return _kilometers;
}

Distance Distance::operator-() const noexcept
{
    return Distance(-_kilometers);
}

bool Distance::operator==(const Distance& other) const noexcept
{
    return _kilometers == other._kilometers;
}

bool Distance::operator!=(const Distance& other) const noexcept
{
    return _kilometers != other._kilometers;
}

bool Distance::operator<(const Distance& other) const noexcept
{
    return _kilometers < other._kilometers;
}

bool Distance::operator<=(const Distance& other) const noexcept
{
    return _kilometers <= other._kilometers;
}

bool Distance::operator>(const Distance& other) const noexcept
{
    return _kilometers > other._kilometers;
}

bool Distance::operator>=(const Distance& other) const noexcept
{
    return _kilometers >= other._kilometers;
}

namespace literals
{

Distance operator""_km(unsigned long long kilometers) noexcept
{
    return Distance::fromKilometers(static_cast<double>(kilometers));
}

Distance operator""_km(long double kilometers) noexcept
{
    return Distance::fromKilometers(static_cast<double>(kilometers));
}

} // namespace literals

} // namespace math
