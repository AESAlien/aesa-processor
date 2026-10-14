#pragma once

#include "math/quantity/quantity_tolerance.hpp"

namespace math
{

class Distance : public Quantity<Distance>
{
public:
    static const QuantityTolerance<Distance> DEFAULT_TOLERANCE;

    static Distance fromMeters(double meters) noexcept;
    static Distance fromKilometers(double kilometers) noexcept;

    double m() const noexcept;
    double km() const noexcept;

    Distance operator-() const noexcept;

private:
    explicit Distance(double meters) noexcept;
};

Distance abs(Distance distance) noexcept;

namespace literals
{

Distance operator""_m(unsigned long long meters) noexcept;
Distance operator""_m(long double meters) noexcept;

Distance operator""_km(unsigned long long kilometers) noexcept;
Distance operator""_km(long double kilometers) noexcept;

}

}
