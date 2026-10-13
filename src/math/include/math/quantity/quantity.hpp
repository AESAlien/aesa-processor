#pragma once

namespace math
{

class Quantity
{
public:
    double value() const noexcept;

    // 저장된 값이 유한하면 true를 반환한다. 무한대·NaN 값은 false를 반환한다.
    bool isFinite() const noexcept;

    // 저장된 값이 0 이상이면 true를 반환한다. NaN 값은 false를 반환한다.
    bool isNonnegative() const noexcept;

protected:
    Quantity(const Quantity&) noexcept;
    Quantity(Quantity&&) noexcept;
    Quantity& operator=(const Quantity&) noexcept;
    Quantity& operator=(Quantity&&) noexcept;

    explicit Quantity(double value) noexcept;

    ~Quantity() noexcept;

    double _value;
};

}
