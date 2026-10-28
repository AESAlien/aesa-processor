#include "math/quantity/length.hpp"

#include <cmath>

namespace math
{
using namespace literals;

const QuantityTolerance<Length> Length::NONE_TOLERANCE{
    0.0_m,
    0.0
};

const QuantityTolerance<Length> Length::DEFAULT_TOLERANCE{
    1e-12_m,
    1e-9
};

Length Length::fromMeters(double meters)
{
    return Length(meters);
}

Length Length::fromKilometers(double kilometers)
{
    return Length(kilometers * 1000.0);
}

double Length::m() const noexcept
{
    return value();
}

double Length::km() const noexcept
{
    return value() / 1000.0;
}

Length::Length(double meters)
    : Quantity<Length>(meters)
{
}

Length abs(Length length) noexcept
{
    return Length::fromMeters(std::abs(length.m()));
}

namespace literals
{

Length operator""_m(unsigned long long meters) noexcept
{
    return Length::fromMeters(static_cast<double>(meters));
}

Length operator""_m(long double meters)
{
    return Length::fromMeters(static_cast<double>(meters));
}

Length operator""_km(unsigned long long kilometers) noexcept
{
    return Length::fromKilometers(static_cast<double>(kilometers));
}

Length operator""_km(long double kilometers)
{
    return Length::fromKilometers(static_cast<double>(kilometers));
}

}

}
