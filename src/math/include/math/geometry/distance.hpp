#pragma once

#include "math/quantity/length.hpp"

namespace math
{

class Distance
{
public:
    explicit Distance(Length length);

    static Distance fromMeters(double meters);
    static Distance fromKilometers(double kilometers);

    double m() const noexcept;
    double km() const noexcept;

    bool equals(
        const Distance& other,
        const QuantityTolerance<Length>& tolerance = Length::DEFAULT_TOLERANCE
    ) const noexcept;
    bool equals(
        Length other,
        const QuantityTolerance<Length>& tolerance = Length::DEFAULT_TOLERANCE
    ) const noexcept;
    bool notEquals(
        const Distance& other,
        const QuantityTolerance<Length>& tolerance = Length::DEFAULT_TOLERANCE
    ) const noexcept;
    bool notEquals(
        Length other,
        const QuantityTolerance<Length>& tolerance = Length::DEFAULT_TOLERANCE
    ) const noexcept;
    bool lessThan(
        const Distance& other,
        const QuantityTolerance<Length>& tolerance = Length::DEFAULT_TOLERANCE
    ) const noexcept;
    bool lessThan(
        Length other,
        const QuantityTolerance<Length>& tolerance = Length::DEFAULT_TOLERANCE
    ) const noexcept;
    bool lessThanOrEqual(
        const Distance& other,
        const QuantityTolerance<Length>& tolerance = Length::DEFAULT_TOLERANCE
    ) const noexcept;
    bool lessThanOrEqual(
        Length other,
        const QuantityTolerance<Length>& tolerance = Length::DEFAULT_TOLERANCE
    ) const noexcept;
    bool greaterThan(
        const Distance& other,
        const QuantityTolerance<Length>& tolerance = Length::DEFAULT_TOLERANCE
    ) const noexcept;
    bool greaterThan(
        Length other,
        const QuantityTolerance<Length>& tolerance = Length::DEFAULT_TOLERANCE
    ) const noexcept;
    bool greaterThanOrEqual(
        const Distance& other,
        const QuantityTolerance<Length>& tolerance = Length::DEFAULT_TOLERANCE
    ) const noexcept;
    bool greaterThanOrEqual(
        Length other,
        const QuantityTolerance<Length>& tolerance = Length::DEFAULT_TOLERANCE
    ) const noexcept;

    Distance operator+(Length offset) const;
    Distance operator-(Length offset) const;
    double operator/(Distance divisor) const;
    Distance& operator+=(Length offset);
    Distance& operator-=(Length offset);

    bool operator==(const Distance& other) const noexcept;
    bool operator!=(const Distance& other) const noexcept;
    bool operator<(const Distance& other) const noexcept;
    bool operator<=(const Distance& other) const noexcept;
    bool operator>(const Distance& other) const noexcept;
    bool operator>=(const Distance& other) const noexcept;
    bool operator==(Length other) const noexcept;
    bool operator!=(Length other) const noexcept;
    bool operator<(Length other) const noexcept;
    bool operator<=(Length other) const noexcept;
    bool operator>(Length other) const noexcept;
    bool operator>=(Length other) const noexcept;

    friend Distance operator+(Length offset, const Distance& distance);
    friend bool operator==(Length length, const Distance& distance) noexcept;
    friend bool operator!=(Length length, const Distance& distance) noexcept;
    friend bool operator<(Length length, const Distance& distance) noexcept;
    friend bool operator<=(Length length, const Distance& distance) noexcept;
    friend bool operator>(Length length, const Distance& distance) noexcept;
    friend bool operator>=(Length length, const Distance& distance) noexcept;

private:
    void validate() const;

    Length _length;
};

}
