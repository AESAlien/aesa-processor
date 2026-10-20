#pragma once

#include <cmath>
#include <stdexcept>

namespace aesa_processor::math
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

    // 저장된 값이 0 이상이면 true를 반환한다.
    bool isNonnegative() const noexcept
    {
        return _value >= 0.0;
    }

    // 지정한 절대오차 또는 상대오차 범위 안이면 true를 반환한다.
    // 허용오차를 생략하면 DEFAULT 절대·상대오차를 적용한다.
    bool equals(const Derived& other, const QuantityTolerance<Derived>& tolerance = Derived::DEFAULT_TOLERANCE) const noexcept
    {
        return tolerance.isWithin(static_cast<const Derived&>(*this), other);
    }

    // 지정한 절대오차와 상대오차 범위 모두 밖이면 true를 반환한다.
    // 허용오차를 생략하면 DEFAULT 절대·상대오차를 적용한다.
    bool notEquals(const Derived& other, const QuantityTolerance<Derived>& tolerance = Derived::DEFAULT_TOLERANCE) const noexcept
    {
        return !equals(other, tolerance);
    }

    // 허용오차를 생략하면 DEFAULT 절대·상대오차를 적용한다.
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

    // DEFAULT 절대오차 또는 상대오차 범위 안이면 true를 반환한다.
    bool operator==(const Derived& other) const noexcept
    {
        return equals(other);
    }

    // DEFAULT 절대오차와 상대오차 범위 모두 밖이면 true를 반환한다.
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

    Derived operator+() const noexcept
    {
        return static_cast<const Derived&>(*this);
    }

    Derived operator-() const noexcept
    {
        return Derived(-_value);
    }

    Derived operator+(const Derived& other) const
    {
        return Derived(_value + other.value());
    }

    Derived operator-(const Derived& other) const
    {
        return Derived(_value - other.value());
    }

    Derived operator*(double scalar) const
    {
        return Derived(_value * scalar);
    }

    Derived operator/(double scalar) const
    {
        if (scalar == 0.0)
        {
            throw std::domain_error("Cannot divide a quantity by zero");
        }
        return Derived(_value / scalar);
    }

    Derived& operator+=(const Derived& other)
    {
        return static_cast<Derived&>(*this) = Derived(_value + other.value());
    }

    Derived& operator-=(const Derived& other)
    {
        return static_cast<Derived&>(*this) = Derived(_value - other.value());
    }

    Derived& operator*=(double scalar)
    {
        return static_cast<Derived&>(*this) = Derived(_value * scalar);
    }

    Derived& operator/=(double scalar)
    {
        if (scalar == 0.0)
        {
            throw std::domain_error("Cannot divide a quantity by zero");
        }
        return static_cast<Derived&>(*this) = Derived(_value / scalar);
    }

    friend Derived operator*(double scalar, const Derived& quantity)
    {
        return quantity * scalar;
    }

protected:
    Quantity(const Quantity&) noexcept = default;
    Quantity(Quantity&&) noexcept = default;
    Quantity& operator=(const Quantity&) noexcept = default;
    Quantity& operator=(Quantity&&) noexcept = default;

    explicit Quantity(double value)
        : _value(value)
    {
        if (!std::isfinite(value))
        {
            throw std::invalid_argument("Quantity must be finite");
        }
    }

    ~Quantity() noexcept = default;

private:
    double _value;
};

}
