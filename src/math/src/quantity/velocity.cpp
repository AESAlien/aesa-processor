#include "math/quantity/velocity.hpp"

#include <cmath>

namespace math
{
using namespace literals;

Velocity Velocity::fromMetersPerSecond(double metersPerSecond) noexcept
{
    return Velocity(metersPerSecond);
}

double Velocity::mps() const noexcept
{
    return _metersPerSecond;
}

bool Velocity::isFinite() const noexcept
{
    return std::isfinite(_metersPerSecond);
}

bool Velocity::isNonnegative() const noexcept
{
    return _metersPerSecond >= 0.0;
}

bool Velocity::equals(const Velocity& other, const Tolerance<Velocity>& tolerance) const noexcept
{
    const Tolerance<double> numericTolerance{tolerance.absolute()._metersPerSecond, tolerance.relative()};
    return numericTolerance.isWithin(_metersPerSecond, other._metersPerSecond);
}

bool Velocity::operator!=(const Velocity& other) const noexcept
{
    return !(*this == other);
}

bool Velocity::operator==(const Velocity& other) const noexcept
{
    // 타입별로 한 번만 초기화해 모든 기본 비교에서 공유한다.
    static const Tolerance<Velocity> DEFAULT_VELOCITY_TOLERANCE{
        fromMetersPerSecond(DEFAULT_TOLERANCE.absolute()),
        DEFAULT_TOLERANCE.relative()
    };
    return equals(other, DEFAULT_VELOCITY_TOLERANCE);
}

bool Velocity::operator<(const Velocity& other) const noexcept
{
    return _metersPerSecond < other._metersPerSecond && *this != other;
}

bool Velocity::operator<=(const Velocity& other) const noexcept
{
    return _metersPerSecond <= other._metersPerSecond || *this == other;
}

bool Velocity::operator>(const Velocity& other) const noexcept
{
    return _metersPerSecond > other._metersPerSecond && *this != other;
}

bool Velocity::operator>=(const Velocity& other) const noexcept
{
    return _metersPerSecond >= other._metersPerSecond || *this == other;
}

Velocity Velocity::operator-() const noexcept
{
    return Velocity(-_metersPerSecond);
}

Velocity::Velocity(double metersPerSecond) noexcept
    : _metersPerSecond(metersPerSecond)
{
}

Velocity abs(Velocity velocity) noexcept
{
    return Velocity::fromMetersPerSecond(std::abs(velocity.mps()));
}

namespace literals
{

Velocity operator""_mps(unsigned long long metersPerSecond) noexcept
{
    return Velocity::fromMetersPerSecond(static_cast<double>(metersPerSecond));
}

Velocity operator""_mps(long double metersPerSecond) noexcept
{
    return Velocity::fromMetersPerSecond(static_cast<double>(metersPerSecond));
}

}

}
