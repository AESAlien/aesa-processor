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
    double _absolute;
    double _relative;
    void validate() const;
    bool isWithinAbsoluteTolerance(double left, double right) const noexcept;
    bool isWithinRelativeTolerance(double left, double right) const noexcept;
};

extern const Tolerance DEFAULT_TOLERANCE;
// 절대·상대 허용오차 없이 비교한다. 무한대·NaN 값은 같다고 판단하지 않는다.
extern const Tolerance NONE_TOLERANCE;

}
