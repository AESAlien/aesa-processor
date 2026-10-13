#pragma once

namespace math
{

// double 값의 절대·상대오차를 검증하고 비교하는 정책 값 객체이다.
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

    bool isWithin(double left, double right) const noexcept;

private:
    bool isWithinAbsoluteTolerance(double left, double right) const noexcept;

    bool isWithinRelativeTolerance(double left, double right) const noexcept;

    void validate() const;

    static bool isFiniteAndNonnegative(double value) noexcept;

    const double _absoluteTolerance;
    const double _relativeTolerance;
};

// 절대오차는 각 타입의 저장 단위에 적용한다.
inline constexpr double DEFAULT_ABSOLUTE_TOLERANCE = 1e-12;
inline constexpr double DEFAULT_RELATIVE_TOLERANCE = 1e-9;
extern const Tolerance DEFAULT_TOLERANCE;

}
