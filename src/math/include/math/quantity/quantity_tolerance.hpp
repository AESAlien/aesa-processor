#pragma once

#include "math/quantity/quantity.hpp"
#include "math/tolerance.hpp"

#include <type_traits>
#include <utility>

namespace math
{

template<typename T>
class QuantityTolerance
{
    static_assert(std::is_base_of_v<Quantity, T>, "T must derive from Quantity");

public:
    // 오차가 음수·무한대·NaN이면 std::invalid_argument를 던진다.
    QuantityTolerance(T absoluteTolerance, double relativeTolerance)
        : _absoluteTolerance(std::move(absoluteTolerance)),
          _numericTolerance(_absoluteTolerance.value(), relativeTolerance)
    {
    }

    const T& absolute() const noexcept
    {
        return _absoluteTolerance;
    }

    double relative() const noexcept
    {
        return _numericTolerance.relative();
    }

    bool isWithin(const T& left, const T& right) const noexcept
    {
        return _numericTolerance.isWithin(left.value(), right.value());
    }

private:
    const T _absoluteTolerance;
    const Tolerance _numericTolerance;
};

}
