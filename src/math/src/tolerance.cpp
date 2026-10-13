#include "math/tolerance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace math
{
namespace
{

bool isFiniteAndNonnegative(double value) noexcept
{
    return value >= 0.0 && value <= std::numeric_limits<double>::max();
}

}  // namespace

const Tolerance DEFAULT_TOLERANCE{
    DEFAULT_ABSOLUTE_TOLERANCE,
    DEFAULT_RELATIVE_TOLERANCE
};

Tolerance::Tolerance(double absoluteTolerance, double relativeTolerance)
    : _absoluteTolerance(absoluteTolerance),
      _relativeTolerance(relativeTolerance)
{
    if (!isFiniteAndNonnegative(_absoluteTolerance) ||
        !isFiniteAndNonnegative(_relativeTolerance)
    ) {
        throw std::invalid_argument("Tolerance values must be finite and non-negative");
    }
}

double Tolerance::absolute() const noexcept
{
    return _absoluteTolerance;
}

double Tolerance::relative() const noexcept
{
    return _relativeTolerance;
}

bool Tolerance::isWithin(double left, double right) const noexcept
{
    if (!std::isfinite(left) || !std::isfinite(right))
    {
        return false;
    }
    return isWithinAbsoluteTolerance(left, right)
        || isWithinRelativeTolerance(left, right);
}

bool Tolerance::isWithinAbsoluteTolerance(double left, double right) const noexcept
{
    return std::abs(left - right) <= _absoluteTolerance;
}

bool Tolerance::isWithinRelativeTolerance(double left, double right) const noexcept
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

}
