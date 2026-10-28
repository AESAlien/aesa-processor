#include "math/geometry/distance.hpp"

#include <cmath>
#include <stdexcept>

namespace math
{
using namespace literals;

Distance::Distance(Length length)
    : _length(length)
{
    validate();
}

void Distance::validate() const
{
    if (!_length.isNonnegative())
    {
        throw std::invalid_argument("Distance must be non-negative");
    }
}

Distance Distance::fromMeters(double meters)
{
    return Distance(Length::fromMeters(meters));
}

Distance Distance::fromKilometers(double kilometers)
{
    return Distance(Length::fromKilometers(kilometers));
}

double Distance::m() const noexcept
{
    return _length.m();
}

double Distance::km() const noexcept
{
    return _length.km();
}

bool Distance::equals(
    const Distance& other,
    const QuantityTolerance<Length>& tolerance
) const noexcept {
    return _length.equals(other._length, tolerance);
}

bool Distance::equals(
    Length other,
    const QuantityTolerance<Length>& tolerance
) const noexcept {
    return _length.equals(other, tolerance);
}

bool Distance::notEquals(
    const Distance& other,
    const QuantityTolerance<Length>& tolerance
) const noexcept {
    return !equals(other, tolerance);
}

bool Distance::notEquals(
    Length other,
    const QuantityTolerance<Length>& tolerance
) const noexcept {
    return !equals(other, tolerance);
}

bool Distance::lessThan(
    const Distance& other,
    const QuantityTolerance<Length>& tolerance
) const noexcept {
    return _length.lessThan(other._length, tolerance);
}

bool Distance::lessThan(
    Length other,
    const QuantityTolerance<Length>& tolerance
) const noexcept {
    return _length.lessThan(other, tolerance);
}

bool Distance::lessThanOrEqual(
    const Distance& other,
    const QuantityTolerance<Length>& tolerance
) const noexcept {
    return _length.lessThanOrEqual(other._length, tolerance);
}

bool Distance::lessThanOrEqual(
    Length other,
    const QuantityTolerance<Length>& tolerance
) const noexcept {
    return _length.lessThanOrEqual(other, tolerance);
}

bool Distance::greaterThan(
    const Distance& other,
    const QuantityTolerance<Length>& tolerance
) const noexcept {
    return _length.greaterThan(other._length, tolerance);
}

bool Distance::greaterThan(
    Length other,
    const QuantityTolerance<Length>& tolerance
) const noexcept {
    return _length.greaterThan(other, tolerance);
}

bool Distance::greaterThanOrEqual(
    const Distance& other,
    const QuantityTolerance<Length>& tolerance
) const noexcept {
    return _length.greaterThanOrEqual(other._length, tolerance);
}

bool Distance::greaterThanOrEqual(
    Length other,
    const QuantityTolerance<Length>& tolerance
) const noexcept {
    return _length.greaterThanOrEqual(other, tolerance);
}

Distance Distance::operator+(Length offset) const
{
    return Distance(_length + offset);
}

Distance Distance::operator-(Length offset) const
{
    return Distance(_length - offset);
}

double Distance::operator/(Distance divisor) const
{
    if (divisor._length.equals(0.0_m, Length::NONE_TOLERANCE))
    {
        throw std::domain_error("Cannot divide a distance by zero");
    }
    const double ratio = _length.value() / divisor._length.value();
    if (!std::isfinite(ratio))
    {
        throw std::overflow_error("Distance ratio is not finite");
    }
    return ratio;
}

Distance& Distance::operator+=(Length offset)
{
    return *this = *this + offset;
}

Distance& Distance::operator-=(Length offset)
{
    return *this = *this - offset;
}

bool Distance::operator==(const Distance& other) const noexcept
{
    return equals(other);
}

bool Distance::operator!=(const Distance& other) const noexcept
{
    return !(*this == other);
}

bool Distance::operator<(const Distance& other) const noexcept
{
    return lessThan(other);
}

bool Distance::operator<=(const Distance& other) const noexcept
{
    return lessThanOrEqual(other);
}

bool Distance::operator>(const Distance& other) const noexcept
{
    return greaterThan(other);
}

bool Distance::operator>=(const Distance& other) const noexcept
{
    return greaterThanOrEqual(other);
}

bool Distance::operator==(Length other) const noexcept
{
    return equals(other);
}

bool Distance::operator!=(Length other) const noexcept
{
    return notEquals(other);
}

bool Distance::operator<(Length other) const noexcept
{
    return lessThan(other);
}

bool Distance::operator<=(Length other) const noexcept
{
    return lessThanOrEqual(other);
}

bool Distance::operator>(Length other) const noexcept
{
    return greaterThan(other);
}

bool Distance::operator>=(Length other) const noexcept
{
    return greaterThanOrEqual(other);
}

Distance operator+(Length offset, const Distance& distance)
{
    return distance + offset;
}

bool operator==(Length length, const Distance& distance) noexcept
{
    return distance == length;
}

bool operator!=(Length length, const Distance& distance) noexcept
{
    return distance.notEquals(length);
}

bool operator<(Length length, const Distance& distance) noexcept
{
    return distance.greaterThan(length);
}

bool operator<=(Length length, const Distance& distance) noexcept
{
    return distance.greaterThanOrEqual(length);
}

bool operator>(Length length, const Distance& distance) noexcept
{
    return distance.lessThan(length);
}

bool operator>=(Length length, const Distance& distance) noexcept
{
    return distance.lessThanOrEqual(length);
}

}
