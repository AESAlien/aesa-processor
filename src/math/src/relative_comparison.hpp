#pragma once

#include <algorithm>
#include <cmath>

namespace math::detail
{

inline bool relativeEquals(double left, double right, double tolerance) noexcept
{
    if (!std::isfinite(left) || !std::isfinite(right) ||
        !std::isfinite(tolerance) || tolerance < 0.0)
    {
        return false;
    }
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
    return relativeDifference <= tolerance;
}

}
