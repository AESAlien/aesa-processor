#pragma once

#include "math/quantity/linear_velocity.hpp"

namespace math
{

class Speed
{
public:
    explicit Speed(LinearVelocity velocity);

    static Speed fromMetersPerSecond(double metersPerSecond);

    double mps() const noexcept;
    LinearVelocity asLinearVelocity() const noexcept;

    bool isPositive() const noexcept;
    bool isNegative() const noexcept;
    bool isNonpositive() const noexcept;
    bool isNonnegative() const noexcept;

    bool equals(
        const Speed& other,
        const QuantityTolerance<LinearVelocity>& tolerance = LinearVelocity::DEFAULT_TOLERANCE
    ) const noexcept;
    bool equals(
        LinearVelocity other,
        const QuantityTolerance<LinearVelocity>& tolerance = LinearVelocity::DEFAULT_TOLERANCE
    ) const noexcept;
    bool notEquals(
        const Speed& other,
        const QuantityTolerance<LinearVelocity>& tolerance = LinearVelocity::DEFAULT_TOLERANCE
    ) const noexcept;
    bool notEquals(
        LinearVelocity other,
        const QuantityTolerance<LinearVelocity>& tolerance = LinearVelocity::DEFAULT_TOLERANCE
    ) const noexcept;
    bool lessThan(
        const Speed& other,
        const QuantityTolerance<LinearVelocity>& tolerance = LinearVelocity::DEFAULT_TOLERANCE
    ) const noexcept;
    bool lessThan(
        LinearVelocity other,
        const QuantityTolerance<LinearVelocity>& tolerance = LinearVelocity::DEFAULT_TOLERANCE
    ) const noexcept;
    bool lessThanOrEqual(
        const Speed& other,
        const QuantityTolerance<LinearVelocity>& tolerance = LinearVelocity::DEFAULT_TOLERANCE
    ) const noexcept;
    bool lessThanOrEqual(
        LinearVelocity other,
        const QuantityTolerance<LinearVelocity>& tolerance = LinearVelocity::DEFAULT_TOLERANCE
    ) const noexcept;
    bool greaterThan(
        const Speed& other,
        const QuantityTolerance<LinearVelocity>& tolerance = LinearVelocity::DEFAULT_TOLERANCE
    ) const noexcept;
    bool greaterThan(
        LinearVelocity other,
        const QuantityTolerance<LinearVelocity>& tolerance = LinearVelocity::DEFAULT_TOLERANCE
    ) const noexcept;
    bool greaterThanOrEqual(
        const Speed& other,
        const QuantityTolerance<LinearVelocity>& tolerance = LinearVelocity::DEFAULT_TOLERANCE
    ) const noexcept;
    bool greaterThanOrEqual(
        LinearVelocity other,
        const QuantityTolerance<LinearVelocity>& tolerance = LinearVelocity::DEFAULT_TOLERANCE
    ) const noexcept;

    Speed operator+(LinearVelocity offset) const;
    Speed operator-(LinearVelocity offset) const;
    double operator/(Speed divisor) const;
    Speed& operator+=(LinearVelocity offset);
    Speed& operator-=(LinearVelocity offset);

    bool operator==(const Speed& other) const noexcept;
    bool operator!=(const Speed& other) const noexcept;
    bool operator<(const Speed& other) const noexcept;
    bool operator<=(const Speed& other) const noexcept;
    bool operator>(const Speed& other) const noexcept;
    bool operator>=(const Speed& other) const noexcept;
    bool operator==(LinearVelocity other) const noexcept;
    bool operator!=(LinearVelocity other) const noexcept;
    bool operator<(LinearVelocity other) const noexcept;
    bool operator<=(LinearVelocity other) const noexcept;
    bool operator>(LinearVelocity other) const noexcept;
    bool operator>=(LinearVelocity other) const noexcept;

    friend Speed operator+(LinearVelocity offset, const Speed& speed);
    friend bool operator==(LinearVelocity velocity, const Speed& speed) noexcept;
    friend bool operator!=(LinearVelocity velocity, const Speed& speed) noexcept;
    friend bool operator<(LinearVelocity velocity, const Speed& speed) noexcept;
    friend bool operator<=(LinearVelocity velocity, const Speed& speed) noexcept;
    friend bool operator>(LinearVelocity velocity, const Speed& speed) noexcept;
    friend bool operator>=(LinearVelocity velocity, const Speed& speed) noexcept;

private:
    LinearVelocity _velocity;
};

}
