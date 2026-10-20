#pragma once

namespace aesa_processor::math
{

struct Tolerance
{
    // 오차가 음수·무한대·NaN이면 std::invalid_argument를 던진다.
    Tolerance(
        double absoluteTolerance,
        double relativeTolerance
    );

    const double absolute;
    const double relative;

    // 지정한 절대오차 또는 상대오차 범위 안이면 true를 반환한다. 무한대·NaN 값은 false를 반환한다.
    bool isWithin(double left, double right) const noexcept;

private:
    bool isWithinAbsoluteTolerance(double left, double right) const noexcept;
    bool isWithinRelativeTolerance(double left, double right) const noexcept;
};

inline constexpr double DEFAULT_ABSOLUTE_TOLERANCE = 1e-12;
inline constexpr double DEFAULT_RELATIVE_TOLERANCE = 1e-9;
extern const Tolerance DEFAULT_TOLERANCE;
// 절대·상대 허용오차 없이 비교한다. 무한대·NaN 값은 같다고 판단하지 않는다.
extern const Tolerance NONE_TOLERANCE;

}
