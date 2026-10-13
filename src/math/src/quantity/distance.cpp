#include "math/quantity/distance.hpp"

#include <cmath>

namespace math
{
using namespace literals;

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
    return _value;
}

double Distance::km() const noexcept
{
    return _value / 1000.0;
}

bool Distance::equals(const Distance& other, const QuantityTolerance<Distance>& tolerance) const noexcept
{
    return tolerance.isWithin(*this, other);
}

bool Distance::operator==(const Distance& other) const noexcept
{
    static const QuantityTolerance<Distance> DEFAULT_DISTANCE_TOLERANCE{
        fromMeters(DEFAULT_TOLERANCE.absolute()),
        DEFAULT_TOLERANCE.relative()
    };

    return equals(other, DEFAULT_DISTANCE_TOLERANCE);
}

bool Distance::operator!=(const Distance& other) const noexcept
{
    return !(*this == other);
}

bool Distance::operator<(const Distance& other) const noexcept
{
    return _value < other._value && *this != other;
}

bool Distance::operator<=(const Distance& other) const noexcept
{
    return _value <= other._value || *this == other;
}

bool Distance::operator>(const Distance& other) const noexcept
{
    return _value > other._value && *this != other;
}

bool Distance::operator>=(const Distance& other) const noexcept
{
    return _value >= other._value || *this == other;
}

Distance Distance::operator-() const noexcept
{
    return Distance(-_value);
}

Distance::Distance(double meters) noexcept
    : Quantity(meters)
{
}

Distance abs(Distance distance) noexcept
{
    return Distance::fromMeters(std::abs(distance.m()));
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
