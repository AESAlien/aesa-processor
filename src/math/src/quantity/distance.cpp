#include "math/quantity/distance.hpp"

#include <cmath>

namespace math
{
using namespace literals;

const QuantityTolerance<Distance> Distance::NONE_TOLERANCE{
    Distance::fromMeters(0.0),
    0.0
};

const QuantityTolerance<Distance> Distance::DEFAULT_TOLERANCE{
    Distance::fromMeters(DEFAULT_ABSOLUTE_TOLERANCE),
    DEFAULT_RELATIVE_TOLERANCE
};

Distance Distance::fromMeters(double meters)
{
    return Distance(meters);
}

Distance Distance::fromKilometers(double kilometers)
{
    return Distance(kilometers * 1000.0);
}

double Distance::m() const noexcept
{
    return value();
}

double Distance::km() const noexcept
{
    return value() / 1000.0;
}

Distance Distance::operator-() const noexcept
{
    return Distance(-value());
}

Distance::Distance(double meters)
    : Quantity<Distance>(meters)
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

Distance operator""_m(long double meters)
{
    return Distance::fromMeters(static_cast<double>(meters));
}

Distance operator""_km(unsigned long long kilometers) noexcept
{
    return Distance::fromKilometers(static_cast<double>(kilometers));
}

Distance operator""_km(long double kilometers)
{
    return Distance::fromKilometers(static_cast<double>(kilometers));
}

}

}
