#pragma once

namespace math
{

class Distance
{
public:
    static Distance fromKilometers(double kilometers) noexcept;

    double km() const noexcept;

    Distance operator-() const noexcept;

    // Compare stored kilometers exactly, without a tolerance.
    bool operator==(const Distance& other) const noexcept;
    bool operator!=(const Distance& other) const noexcept;
    bool operator<(const Distance& other) const noexcept;
    bool operator<=(const Distance& other) const noexcept;
    bool operator>(const Distance& other) const noexcept;
    bool operator>=(const Distance& other) const noexcept;

private:
    explicit Distance(double kilometers) noexcept;

    double _kilometers = 0.0;
};

namespace literals
{

Distance operator""_km(unsigned long long kilometers) noexcept;
Distance operator""_km(long double kilometers) noexcept;

} // namespace literals

} // namespace math
