#pragma once

#include <cmath>

namespace math
{

template<typename T>
class QuantityTolerance;

template<typename Derived>
class Quantity
{
public:
    double value() const noexcept
    {
        return _value;
    }

    // 저장된 값이 유한하면 true를 반환한다. 무한대·NaN 값은 false를 반환한다.
    bool isFinite() const noexcept
    {
        return std::isfinite(_value);
    }

    // 저장된 값이 0 이상이면 true를 반환한다. NaN 값은 false를 반환한다.
    bool isNonnegative() const noexcept
    {
        return _value >= 0.0;
    }

    // 지정한 절대오차 또는 상대오차 범위 안이면 true를 반환한다. 무한대·NaN 값은 false를 반환한다.
    // 허용오차를 생략하면 DEFAULT 절대·상대오차를 적용한다.
    bool equals(const Derived& other, const QuantityTolerance<Derived>& tolerance = Derived::DEFAULT_TOLERANCE) const noexcept
    {
        return tolerance.isWithin(static_cast<const Derived&>(*this), other);
    }

    // 지정한 절대오차와 상대오차 범위 모두 밖이면 true를 반환한다. 무한대·NaN 값은 true를 반환한다.
    // 허용오차를 생략하면 DEFAULT 절대·상대오차를 적용한다.
    bool notEquals(const Derived& other, const QuantityTolerance<Derived>& tolerance = Derived::DEFAULT_TOLERANCE) const noexcept
    {
        return !equals(other, tolerance);
    }

    // 허용오차를 생략하면 DEFAULT 절대·상대오차를 적용한다.
    // 무한대·NaN의 순서는 double 비교 규칙을 따른다.
    // 지정한 오차 범위 밖에서 더 작으면 true를 반환한다.
    bool lessThan(const Derived& other, const QuantityTolerance<Derived>& tolerance = Derived::DEFAULT_TOLERANCE) const noexcept
    {
        return _value < other.value() && !equals(other, tolerance);
    }

    // 더 작거나 지정한 오차 범위 안이면 true를 반환한다.
    bool lessThanOrEqual(const Derived& other, const QuantityTolerance<Derived>& tolerance = Derived::DEFAULT_TOLERANCE) const noexcept
    {
        return _value <= other.value() || equals(other, tolerance);
    }

    // 지정한 오차 범위 밖에서 더 크면 true를 반환한다.
    bool greaterThan(const Derived& other, const QuantityTolerance<Derived>& tolerance = Derived::DEFAULT_TOLERANCE) const noexcept
    {
        return _value > other.value() && !equals(other, tolerance);
    }

    // 더 크거나 지정한 오차 범위 안이면 true를 반환한다.
    bool greaterThanOrEqual(const Derived& other, const QuantityTolerance<Derived>& tolerance = Derived::DEFAULT_TOLERANCE) const noexcept
    {
        return _value >= other.value() || equals(other, tolerance);
    }

    // DEFAULT 절대오차 또는 상대오차 범위 안이면 true를 반환한다. 무한대·NaN 값은 false를 반환한다.
    bool operator==(const Derived& other) const noexcept
    {
        return equals(other);
    }

    // DEFAULT 절대오차와 상대오차 범위 모두 밖이면 true를 반환한다. 무한대·NaN 값은 true를 반환한다.
    bool operator!=(const Derived& other) const noexcept
    {
        return notEquals(other);
    }

    // DEFAULT 오차 범위 밖에서 더 작으면 true를 반환한다.
    bool operator<(const Derived& other) const noexcept
    {
        return lessThan(other);
    }

    // 더 작거나 DEFAULT 오차 범위 안이면 true를 반환한다.
    bool operator<=(const Derived& other) const noexcept
    {
        return lessThanOrEqual(other);
    }

    // DEFAULT 오차 범위 밖에서 더 크면 true를 반환한다.
    bool operator>(const Derived& other) const noexcept
    {
        return greaterThan(other);
    }

    // 더 크거나 DEFAULT 오차 범위 안이면 true를 반환한다.
    bool operator>=(const Derived& other) const noexcept
    {
        return greaterThanOrEqual(other);
    }

protected:
    Quantity(const Quantity&) noexcept = default;
    Quantity(Quantity&&) noexcept = default;
    Quantity& operator=(const Quantity&) noexcept = default;
    Quantity& operator=(Quantity&&) noexcept = default;

    explicit Quantity(double value) noexcept
        : _value(value)
    {
    }

    ~Quantity() noexcept = default;

    double _value;
};

}
