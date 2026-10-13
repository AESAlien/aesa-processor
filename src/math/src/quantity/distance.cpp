#include "math/quantity/distance.hpp"

#include <cmath>

namespace math
{
using namespace literals;

Distance Distance::fromMeters(double meters) noexcept
{
    return Distance(meters);
}

Distance Distance::fromKilometers(double kilometers) noexcept
{
    return Distance(kilometers * 1000.0);
}

double Distance::m() const noexcept
{
    return _meters;
}

double Distance::km() const noexcept
{
    return _meters / 1000.0;
}

bool Distance::isFinite() const noexcept
{
    return std::isfinite(_meters);
}

bool Distance::isNonnegative() const noexcept
{
    return _meters >= 0.0;
}

bool Distance::equals(const Distance& other, const Tolerance<Distance>& tolerance) const noexcept
{
    const Tolerance<double> numericTolerance{tolerance.absolute()._meters, tolerance.relative()};
    return numericTolerance.isWithin(_meters, other._meters);
}

bool Distance::operator!=(const Distance& other) const noexcept
{
    return !(*this == other);
}

bool Distance::operator==(const Distance& other) const noexcept
{
    // 타입별로 한 번만 초기화해 모든 기본 비교에서 공유한다.
    static const Tolerance<Distance> DEFAULT_DISTANCE_TOLERANCE{
        fromMeters(DEFAULT_TOLERANCE.absolute()),
        DEFAULT_TOLERANCE.relative()
    };
    return equals(other, DEFAULT_DISTANCE_TOLERANCE);
}

bool Distance::operator<(const Distance& other) const noexcept
{
    return _meters < other._meters && *this != other;
}

bool Distance::operator<=(const Distance& other) const noexcept
{
    return _meters <= other._meters || *this == other;
}

bool Distance::operator>(const Distance& other) const noexcept
{
    return _meters > other._meters && *this != other;
}

bool Distance::operator>=(const Distance& other) const noexcept
{
    return _meters >= other._meters || *this == other;
}

Distance Distance::operator-() const noexcept
{
    return Distance(-_meters);
}

Distance::Distance(double meters) noexcept
    : _meters(meters)
{
}

Distance abs(Distance distance) noexcept
{
    return Distance::fromMeters(std::abs(distance.m()));
}

namespace literals
{

Distance operator""_m(unsigned long long meters) noexcept
{
    return Distance::fromMeters(static_cast<double>(meters));
}

Distance operator""_m(long double meters) noexcept
{
    return Distance::fromMeters(static_cast<double>(meters));
}

Distance operator""_km(unsigned long long kilometers) noexcept
{
    return Distance::fromKilometers(static_cast<double>(kilometers));
}

Distance operator""_km(long double kilometers) noexcept
{
    return Distance::fromKilometers(static_cast<double>(kilometers));
}

}

}
