#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace math
{

// 절대오차의 타입·단위를 보존하는 비교 정책 값 객체이다.
// 생성 시 모든 오차를 검증한다. 비교 대상과의 차원 일치는 equals가 검사한다.
template<typename T>
class Tolerance
{
public:
    // 오차가 음수·무한대·NaN이면 std::invalid_argument를 던진다.
    constexpr Tolerance(
        T absoluteTolerance,
        double relativeTolerance
    )
        : _absoluteTolerance(std::move(absoluteTolerance)),
          _relativeTolerance(relativeTolerance)
    {
        validate();
    }

    constexpr const T& absolute() const noexcept
    {
        return _absoluteTolerance;
    }

    constexpr double relative() const noexcept
    {
        return _relativeTolerance;
    }

    // double 정책으로 두 유한한 수가 절대오차 또는 상대오차 안인지 판정한다.
    template<typename U = T>
    std::enable_if_t<std::is_same_v<U, T> && std::is_same_v<U, double>, bool>
    isWithin(double left, double right) const noexcept
    {
        if (!std::isfinite(left) || !std::isfinite(right))
        {
            return false;
        }
        return std::abs(left - right) <= _absoluteTolerance
            || isWithinRelativeTolerance(left, right);
    }

private:
    bool isWithinRelativeTolerance(double left, double right) const noexcept
    {
        if (left == right)
        {
            return true;
        }
        const double difference = std::abs(left - right);
        const double scale = std::max(std::abs(left), std::abs(right));
        // 차가 오버플로하면 각각을 정규화한 뒤 상대 차이를 구한다.
        const double relativeDifference = std::isfinite(difference)
            ? difference / scale
            : std::abs(left / scale - right / scale);
        return relativeDifference <= _relativeTolerance;
    }

    constexpr void validate() const
    {
        if (!isFiniteAndNonnegative(_absoluteTolerance) ||
            !isFiniteAndNonnegative(_relativeTolerance)
        ) {
            throw std::invalid_argument("Tolerance values must be finite and non-negative");
        }
    }

    template<typename U>
    static constexpr bool isFiniteAndNonnegative(const U& value) noexcept
    {
        if constexpr (std::is_arithmetic_v<U>)
        {
            return value >= 0.0 && value <= std::numeric_limits<double>::max();
        }
        else
        {
            return value.isFinite() && value.isNonnegative();
        }
    }

    const T _absoluteTolerance;
    const double _relativeTolerance;
};

// 절대오차는 각 타입의 저장 단위에 적용한다.
inline constexpr double DEFAULT_ABSOLUTE_TOLERANCE = 1e-12;
inline constexpr double DEFAULT_RELATIVE_TOLERANCE = 1e-9;
inline constexpr Tolerance<double> DEFAULT_TOLERANCE{
    DEFAULT_ABSOLUTE_TOLERANCE,
    DEFAULT_RELATIVE_TOLERANCE
};

}
