#pragma once

#include "math/quantity/quantity_tolerance.hpp"

namespace math
{

class Length : public Quantity<Length>
{
public:
    static const QuantityTolerance<Length> DEFAULT_TOLERANCE;
    // 절대·상대 허용오차 없이 비교한다.
    static const QuantityTolerance<Length> NONE_TOLERANCE;

    // 저장된 값이 유한하지 않으면 std::invalid_argument를 던진다.
    static Length fromMeters(double meters);
    // 저장된 값이 유한하지 않으면 std::invalid_argument를 던진다.
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
