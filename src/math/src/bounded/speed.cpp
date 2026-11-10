#include "math/bounded/speed.hpp"

#include <cmath>
#include <stdexcept>

namespace math
{

Speed::Speed(LinearVelocity velocity)
  : _velocity(velocity)
{
    if (_velocity.isNegative())
    {
        throw std::invalid_argument("Speed must be non-negative");
    }
}

Speed Speed::fromMetersPerSecond(double metersPerSecond)
{
    return Speed(LinearVelocity::fromMetersPerSecond(metersPerSecond));
}

double Speed::mps() const noexcept
{
    return _velocity.mps();
}

LinearVelocity Speed::asLinearVelocity() const noexcept
{
    return _velocity;
}

bool Speed::isPositive() const noexcept
{
    return _velocity.isPositive();
}

bool Speed::isNegative() const noexcept
{
    return _velocity.isNegative();
}

bool Speed::isNonpositive() const noexcept
{
    return _velocity.isNonpositive();
}

bool Speed::isNonnegative() const noexcept
{
    return _velocity.isNonnegative();
}

bool Speed::equals(
    const Speed& other,
    const QuantityTolerance<LinearVelocity>& tolerance
) const noexcept {
    return equals(other._velocity, tolerance);
}

bool Speed::equals(
    LinearVelocity other,
    const QuantityTolerance<LinearVelocity>& tolerance
) const noexcept {
    return _velocity.equals(other, tolerance);
}

bool Speed::notEquals(
    const Speed& other,
    const QuantityTolerance<LinearVelocity>& tolerance
) const noexcept {
    return notEquals(other._velocity, tolerance);
}

bool Speed::notEquals(
    LinearVelocity other,
    const QuantityTolerance<LinearVelocity>& tolerance
) const noexcept {
    return _velocity.notEquals(other, tolerance);
}

bool Speed::lessThan(
    const Speed& other,
    const QuantityTolerance<LinearVelocity>& tolerance
) const noexcept {
    return lessThan(other._velocity, tolerance);
}

bool Speed::lessThan(
    LinearVelocity other,
    const QuantityTolerance<LinearVelocity>& tolerance
) const noexcept {
    return _velocity.lessThan(other, tolerance);
}

bool Speed::lessThanOrEqual(
    const Speed& other,
    const QuantityTolerance<LinearVelocity>& tolerance
) const noexcept {
    return lessThanOrEqual(other._velocity, tolerance);
}

bool Speed::lessThanOrEqual(
    LinearVelocity other,
    const QuantityTolerance<LinearVelocity>& tolerance
) const noexcept {
    return _velocity.lessThanOrEqual(other, tolerance);
}

bool Speed::greaterThan(
    const Speed& other,
    const QuantityTolerance<LinearVelocity>& tolerance
) const noexcept {
    return greaterThan(other._velocity, tolerance);
}

bool Speed::greaterThan(
    LinearVelocity other,
    const QuantityTolerance<LinearVelocity>& tolerance
) const noexcept {
    return _velocity.greaterThan(other, tolerance);
}

bool Speed::greaterThanOrEqual(
    const Speed& other,
    const QuantityTolerance<LinearVelocity>& tolerance
) const noexcept {
    return greaterThanOrEqual(other._velocity, tolerance);
}

bool Speed::greaterThanOrEqual(
    LinearVelocity other,
    const QuantityTolerance<LinearVelocity>& tolerance
) const noexcept {
    return _velocity.greaterThanOrEqual(other, tolerance);
}

Speed Speed::operator+(LinearVelocity offset) const
{
    return Speed(_velocity + offset);
}

Speed Speed::operator-(LinearVelocity offset) const
{
    return Speed(_velocity - offset);
}

double Speed::operator/(Speed divisor) const
{
    if (divisor.mps() == 0.0)
    {
        throw std::domain_error("Cannot divide a speed by zero");
    }
    const double ratio = mps() / divisor.mps();
    if (!std::isfinite(ratio))
    {
        throw std::overflow_error("Speed ratio is not finite");
    }
    return ratio;
}

Speed& Speed::operator+=(LinearVelocity offset)
{
    return *this = *this + offset;
}

Speed& Speed::operator-=(LinearVelocity offset)
{
    return *this = *this - offset;
}

bool Speed::operator==(const Speed& other) const noexcept
{
    return equals(other);
}

bool Speed::operator!=(const Speed& other) const noexcept
{
    return notEquals(other);
}

bool Speed::operator<(const Speed& other) const noexcept
{
    return lessThan(other);
}

bool Speed::operator<=(const Speed& other) const noexcept
{
    return lessThanOrEqual(other);
}

bool Speed::operator>(const Speed& other) const noexcept
{
    return greaterThan(other);
}

bool Speed::operator>=(const Speed& other) const noexcept
{
    return greaterThanOrEqual(other);
}

bool Speed::operator==(LinearVelocity other) const noexcept
{
    return equals(other);
}

bool Speed::operator!=(LinearVelocity other) const noexcept
{
    return notEquals(other);
}

bool Speed::operator<(LinearVelocity other) const noexcept
{
    return lessThan(other);
}

bool Speed::operator<=(LinearVelocity other) const noexcept
{
    return lessThanOrEqual(other);
}

bool Speed::operator>(LinearVelocity other) const noexcept
{
    return greaterThan(other);
}

bool Speed::operator>=(LinearVelocity other) const noexcept
{
    return greaterThanOrEqual(other);
}

Speed operator+(LinearVelocity offset, const Speed& speed)
{
    return speed + offset;
}

bool operator==(LinearVelocity velocity, const Speed& speed) noexcept
{
    return speed.equals(velocity);
}

bool operator!=(LinearVelocity velocity, const Speed& speed) noexcept
{
    return speed.notEquals(velocity);
}

bool operator<(LinearVelocity velocity, const Speed& speed) noexcept
{
    return speed.greaterThan(velocity);
}

bool operator<=(LinearVelocity velocity, const Speed& speed) noexcept
{
    return speed.greaterThanOrEqual(velocity);
}

bool operator>(LinearVelocity velocity, const Speed& speed) noexcept
{
    return speed.lessThan(velocity);
}

bool operator>=(LinearVelocity velocity, const Speed& speed) noexcept
{
    return speed.lessThanOrEqual(velocity);
}

}
