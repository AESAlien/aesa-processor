#pragma once

#include "math/quantity/quantity_tolerance.hpp"

namespace math
{

class Distance : public Quantity<Distance>
{
public:
    static const QuantityTolerance<Distance> DEFAULT_TOLERANCE;
    // 절대·상대 허용오차 없이 비교한다. 무한대·NaN 값은 같다고 판단하지 않는다.
    static const QuantityTolerance<Distance> NONE_TOLERANCE;

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
