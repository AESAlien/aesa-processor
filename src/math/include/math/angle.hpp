#pragma once

namespace math
{

extern const double PI;

class Angle
{
public:
    static Angle fromDegrees(double degrees) noexcept;
    static Angle fromRadians(double radians) noexcept;

    double deg() const noexcept;
    double rad() const noexcept;

    // Return an angle in [-180, 180) degrees without changing this angle.
    Angle wrap180() const noexcept;

    Angle operator+() const noexcept;
    Angle operator-() const noexcept;
    Angle operator+(const Angle& other) const noexcept;
    Angle operator-(const Angle& other) const noexcept;
    Angle operator*(double scalar) const noexcept;
    Angle operator/(double scalar) const;

    Angle& operator+=(const Angle& other) noexcept;
    Angle& operator-=(const Angle& other) noexcept;
    Angle& operator*=(double scalar) noexcept;
    Angle& operator/=(double scalar);

    // Compare stored degrees exactly, without wrapping or a tolerance.
    bool operator==(const Angle& other) const noexcept;
    bool operator!=(const Angle& other) const noexcept;
    bool operator<(const Angle& other) const noexcept;
    bool operator<=(const Angle& other) const noexcept;
    bool operator>(const Angle& other) const noexcept;
    bool operator>=(const Angle& other) const noexcept;

private:
    explicit Angle(double degrees) noexcept;

    double _degrees = 0.0;
};

Angle operator*(double scalar, const Angle& angle) noexcept;

namespace literals
{

Angle operator""_deg(unsigned long long degrees) noexcept;
Angle operator""_deg(long double degrees) noexcept;

} // namespace literals

} // namespace math
