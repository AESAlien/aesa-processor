#include "math/quantity/quantity.hpp"

#include <cmath>

namespace math
{

Quantity::Quantity(const Quantity&) noexcept = default;
Quantity::Quantity(Quantity&&) noexcept = default;
Quantity& Quantity::operator=(const Quantity&) noexcept = default;
Quantity& Quantity::operator=(Quantity&&) noexcept = default;
Quantity::~Quantity() noexcept = default;

double Quantity::value() const noexcept
{
    return _value;
}

bool Quantity::isFinite() const noexcept
{
    return std::isfinite(_value);
}

bool Quantity::isNonnegative() const noexcept
{
    return _value >= 0.0;
}

Quantity::Quantity(double value) noexcept
    : _value(value)
{
}

}
