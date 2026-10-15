#pragma once

#include "math/quantity/quantity_tolerance.hpp"

namespace math
{

class Length : public Quantity<Length>
{
public:
    static const QuantityTolerance<Length> DEFAULT_TOLERANCE;
    // Compares without absolute or relative tolerance.
    static const QuantityTolerance<Length> NONE_TOLERANCE;

    // Rejects non-finite stored values with std::invalid_argument.
    static Length fromMeters(double meters);
    static Length fromKilometers(double kilometers);

    double m() const noexcept;
    double km() const noexcept;

    Length operator-() const noexcept;

private:
    explicit Length(double meters);
};

Length abs(Length length) noexcept;

namespace literals
{

Length operator""_m(unsigned long long meters) noexcept;
Length operator""_m(long double meters);

Length operator""_km(unsigned long long kilometers) noexcept;
Length operator""_km(long double kilometers);

}

}
