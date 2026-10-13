#pragma once

namespace math
{

class Tolerance
{
public:
    // 오차가 음수·무한대·NaN이면 std::invalid_argument를 던진다.
    Tolerance(
        double absoluteTolerance,
        double relativeTolerance
    );

    double absolute() const noexcept;
    double relative() const noexcept;

    // 지정한 절대오차 또는 상대오차 범위 안이면 true를 반환한다. 무한대·NaN 값은 false를 반환한다.
    bool isWithin(double left, double right) const noexcept;

private:
    bool isWithinAbsoluteTolerance(double left, double right) const noexcept;
    bool isWithinRelativeTolerance(double left, double right) const noexcept;

    const double _absoluteTolerance;
    const double _relativeTolerance;
};

inline constexpr double DEFAULT_ABSOLUTE_TOLERANCE = 1e-12;
inline constexpr double DEFAULT_RELATIVE_TOLERANCE = 1e-9;
extern const Tolerance DEFAULT_TOLERANCE;

}
